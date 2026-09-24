# Igual que el triplet x64-windows de serie, mas un piso de politicas para
# CMake 4.x: hidapi 0.14.0 declara cmake_minimum_required(< 3.5) y CMake 4
# elimino la compatibilidad con eso. Con CMAKE_POLICY_VERSION_MINIMUM=3.5
# se trata como 3.5 (con warning en vez de error fatal). Para los demas
# puertos (que ya piden >= 3.5) esta variable no hace nada.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DCMAKE_POLICY_VERSION_MINIMUM=3.5")
