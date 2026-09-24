// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#ifndef CHIAKI_CONGESTIONCONTROL_H
#define CHIAKI_CONGESTIONCONTROL_H

#include "takion.h"
#include "thread.h"
#include "packetstats.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct chiaki_congestion_control_t
{
	ChiakiTakion *takion;
	ChiakiPacketStats *stats;
	ChiakiThread thread;
	ChiakiBoolPredCond stop_cond;
	double packet_loss;
	double packet_loss_max;
	/* Automatic honest loss reporting: when the measured loss keeps exceeding
	 * packet_loss_max, clamping is bypassed so the PS5's own adaptive bitrate
	 * reacts to the real loss (graceful degradation instead of artifacts).
	 * Hysteresis on both transitions avoids flapping. */
	bool auto_honest_reporting;
	unsigned int loss_over_count;
	unsigned int loss_under_count;
} ChiakiCongestionControl;

CHIAKI_EXPORT ChiakiErrorCode chiaki_congestion_control_start(ChiakiCongestionControl *control, ChiakiTakion *takion, ChiakiPacketStats *stats, double packet_loss_max);

/**
 * Stop control and join the thread
 */
CHIAKI_EXPORT ChiakiErrorCode chiaki_congestion_control_stop(ChiakiCongestionControl *control);

#ifdef __cplusplus
}
#endif

#endif // CHIAKI_CONGESTIONCONTROL_H
