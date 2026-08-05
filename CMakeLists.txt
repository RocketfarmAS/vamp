cmake_minimum_required(VERSION 3.16)
project(vamp)

# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 (ament) packaging of VAMP for the Sentinel stack — the `ros2` branch.
# This CMakeLists REPLACES upstream's (which drives the CPM/nanobind python
# build); see ROS2_PACKAGE.md for the branch contract. Consumers link the
# exported `vamp::vamp` interface target and get the headers, the pinned
# transitive deps (nigh, pdqsort, simdxorshift), the compiled simdxorshift
# static library, and the SIMD baseline flags.
# ──────────────────────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# x86-64 SIMD baseline for vamp's AVX kernels. Fixed (NOT -march=native) so the
# baseline is identical across dev machines and CI. Exported as INTERFACE compile
# options: vamp's kernels compile in the consuming TU, so the consumer must build
# with the same baseline (this mirrors the previous in-tree vendoring exactly).
set(VAMP_SIMD_FLAGS -O3 -mavx2 -mfma)

# simdxorshift: the only dep with compiled sources (AVX2 xorshift128+ RNG backing
# vamp/random/xorshift.hh). -w silences vendored C warnings.
add_library(vamp_simdxorshift STATIC
  external/simdxorshift/src/simdxorshift128plus.c
  external/simdxorshift/src/xorshift128plus.c
)
set_target_properties(vamp_simdxorshift PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vamp_simdxorshift SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/simdxorshift/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}_external>"
)
target_compile_options(vamp_simdxorshift PRIVATE ${VAMP_SIMD_FLAGS} -w)

# vamp core is header-only -> INTERFACE. Headers are SYSTEM so consumer linters
# and -Wall/-Wextra never fire on this external code. All include roots collapse
# into one installed dir so the include forms are identical to upstream usage:
#   #include <vamp/...>  #include <nigh/...>  #include <pdqsort.h>
#   #include <simdxorshift128plus.h>
add_library(vamp INTERFACE)
target_include_directories(vamp SYSTEM INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/impl>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/nigh/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/pdqsort>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}_external>"
)
target_compile_features(vamp INTERFACE cxx_std_17)
target_compile_options(vamp INTERFACE ${VAMP_SIMD_FLAGS})
# Consumers compile vamp kernels into their own TUs, often inside static libs that
# feed shared plugin libraries (vamp_sentinel_bridge) — force PIC downstream, as
# the previous in-tree vendoring did.
set_target_properties(vamp PROPERTIES INTERFACE_POSITION_INDEPENDENT_CODE ON)
target_link_libraries(vamp INTERFACE Eigen3::Eigen vamp_simdxorshift)

# One install root (include/vamp_external) holding every header tree, preserving
# the upstream include forms above. The vamp/ headers live under it too.
install(DIRECTORY src/impl/vamp
  DESTINATION include/${PROJECT_NAME}_external
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.h" PATTERN "*.hpp"
  PATTERN "bindings" EXCLUDE
)
install(DIRECTORY external/nigh/include/nigh
  DESTINATION include/${PROJECT_NAME}_external
)
install(FILES external/pdqsort/pdqsort.h
  DESTINATION include/${PROJECT_NAME}_external
)
install(DIRECTORY external/simdxorshift/include/
  DESTINATION include/${PROJECT_NAME}_external
)
install(FILES LICENSE.txt DESTINATION share/${PROJECT_NAME})
install(FILES external/nigh/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/nigh
)
install(FILES external/pdqsort/license.txt
  DESTINATION share/${PROJECT_NAME}/licenses/pdqsort
)
install(FILES external/simdxorshift/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/simdxorshift
)

install(TARGETS vamp vamp_simdxorshift
  EXPORT export_vamp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_vamp HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
