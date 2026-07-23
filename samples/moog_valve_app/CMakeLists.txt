cmake_minimum_required(VERSION 3.16)

# Project name + language
project(moog_valve_app C)

# ---- Options ----
# Point this at the SOEM repo root (default assumes: SOEM/samples/moog_valve_app)
set(SOEM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "Path to SOEM repository root")

# ---- Executable ----
add_executable(moog_valve_app
  moog2.c
)

# ---- Include paths ----
# Code includes "soem/soem.h", which lives at ${SOEM_ROOT}/include/soem/soem.h
# ec_options.h is generated into the build tree.
target_include_directories(moog_valve_app PRIVATE
  "${SOEM_ROOT}/include"
  "${SOEM_ROOT}/build/include"
  "${SOEM_ROOT}/osal"
  "${SOEM_ROOT}/osal/linux"
  "${SOEM_ROOT}/oshw/linux"
)
# ---- SOEM library ----
# SOEM 2.x is already built as a static library; link it rather than
# recompiling its sources. There are no top-level osal/ or oshw/ dirs
# in this layout - the NIC driver is inside libsoem.a.
find_library(SOEM_LIB
  NAMES soem
  PATHS "${SOEM_ROOT}/build"
  NO_DEFAULT_PATH
  REQUIRED
)

# ---- Compile options ----
target_compile_features(moog_valve_app PRIVATE c_std_99)
target_compile_options(moog_valve_app PRIVATE
  -O2 -Wall -Wextra
)

# ---- Link libs ----
find_package(Threads REQUIRED)
target_link_libraries(moog_valve_app PRIVATE
  ${SOEM_LIB}
  Threads::Threads
)

# Some systems need librt for clock/time; harmless if not needed.
# (On newer glibc it's often unnecessary.)
target_link_libraries(moog_valve_app PRIVATE rt)

# ---- Output ----
set_target_properties(moog_valve_app PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
