// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include <chiaki/congestioncontrol.h>

#define CONGESTION_CONTROL_INTERVAL_MS 200

/* Automatic honest loss reporting: when the measured loss keeps exceeding the
 * configured packet_loss_max, the report clamp is bypassed so the PS5's own
 * adaptive bitrate reacts to the real loss (graceful degradation instead of
 * artifacts). Hysteresis on both transitions avoids flapping. A packet_loss_max
 * of 0 (explicit "never report loss") or >= 1 (already honest) disables this. */
#define AUTO_HONEST_TRIGGER_INTERVALS 5   /* 1.0 s of sustained excess loss */
#define AUTO_HONEST_RECOVER_INTERVALS 10  /* 2.0 s of clean link to re-engage the clamp */

static void *congestion_control_thread_func(void *user)
{
	ChiakiCongestionControl *control = user;
	chiaki_thread_set_affinity(CHIAKI_THREAD_NAME_CONGESTION);

	ChiakiErrorCode err = chiaki_bool_pred_cond_lock(&control->stop_cond);
	if(err != CHIAKI_ERR_SUCCESS)
		return NULL;

	while(true)
	{
		err = chiaki_bool_pred_cond_timedwait(&control->stop_cond, CONGESTION_CONTROL_INTERVAL_MS);
		if(err != CHIAKI_ERR_TIMEOUT)
			break;

		uint64_t received;
		uint64_t lost;
		chiaki_packet_stats_get(control->stats, true, &received, &lost);
		ChiakiTakionCongestionPacket packet = { 0 };
		uint64_t total = received + lost;
		control->packet_loss = total > 0 ? (double)lost / total : 0;

		bool auto_honest_available = control->packet_loss_max > 0.0 && control->packet_loss_max < 1.0;
		if(auto_honest_available)
		{
			if(control->packet_loss > control->packet_loss_max)
			{
				control->loss_under_count = 0;
				if(!control->auto_honest_reporting && ++control->loss_over_count >= AUTO_HONEST_TRIGGER_INTERVALS)
				{
					control->auto_honest_reporting = true;
					CHIAKI_LOGI(control->takion->log, "Sustained packet loss above reported max (%.1f%% > %.1f%%): reporting honest loss so the PS5 can adapt its bitrate",
						control->packet_loss * 100.0, control->packet_loss_max * 100.0);
				}
			}
			else
			{
				control->loss_over_count = 0;
				if(control->auto_honest_reporting && ++control->loss_under_count >= AUTO_HONEST_RECOVER_INTERVALS)
				{
					control->auto_honest_reporting = false;
					control->loss_under_count = 0;
					CHIAKI_LOGI(control->takion->log, "Link recovered: re-engaging packet loss report cap of %.1f%%",
						control->packet_loss_max * 100.0);
				}
			}
		}

		if(!control->auto_honest_reporting && control->packet_loss > control->packet_loss_max)
		{
			CHIAKI_LOGD(control->takion->log, "Clamping reported packet loss: measured=%.1f%% reported_max=%.1f%%",
				control->packet_loss * 100.0, control->packet_loss_max * 100.0);
			lost = (uint64_t)(total * control->packet_loss_max);
			received = total - lost;
		}
		packet.received = (uint16_t)received;
		packet.lost = (uint16_t)lost;
		CHIAKI_LOGV(control->takion->log, "Sending Congestion Control Packet, received: %u, lost: %u",
			(unsigned int)packet.received, (unsigned int)packet.lost);
		chiaki_takion_send_congestion(control->takion, &packet);
	}

	chiaki_bool_pred_cond_unlock(&control->stop_cond);
	return NULL;
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_congestion_control_start(ChiakiCongestionControl *control, ChiakiTakion *takion, ChiakiPacketStats *stats, double packet_loss_max)
{
	control->takion = takion;
	control->stats = stats;
	control->packet_loss_max = packet_loss_max;
	control->packet_loss = 0;
	control->auto_honest_reporting = false;
	control->loss_over_count = 0;
	control->loss_under_count = 0;

	ChiakiErrorCode err = chiaki_bool_pred_cond_init(&control->stop_cond);
	if(err != CHIAKI_ERR_SUCCESS)
		return err;

	err = chiaki_thread_create(&control->thread, congestion_control_thread_func, control);
	if(err != CHIAKI_ERR_SUCCESS)
	{
		chiaki_bool_pred_cond_fini(&control->stop_cond);
		return err;
	}

	chiaki_thread_set_name(&control->thread, "Chiaki Congestion Control");

	return CHIAKI_ERR_SUCCESS;
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_congestion_control_stop(ChiakiCongestionControl *control)
{
	ChiakiErrorCode err = chiaki_bool_pred_cond_signal(&control->stop_cond);
	if(err != CHIAKI_ERR_SUCCESS)
		return err;

	err = chiaki_thread_join(&control->thread, NULL);
	if(err != CHIAKI_ERR_SUCCESS)
		return err;
	control->thread.thread = 0;

	return chiaki_bool_pred_cond_fini(&control->stop_cond);
}
