cmake_minimum_required(VERSION 3.20)

project(WingSimulatorDriver CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_OSX_ARCHITECTURES)
  # coreaudiod's driver host is arm64e/arm64 on Apple silicon, x86_64 on Intel.
  set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "" FORCE)
endif()
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "")

# Ad-hoc ("-") signing by default; set to a Developer ID to sign for real.
set(CODESIGN_ID "-" CACHE STRING "codesign identity for the .driver bundle")

# ---- libASPL (MIT) via FetchContent, pinned ----
include(FetchContent)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(libASPL
  GIT_REPOSITORY https://github.com/gavv/libASPL.git
  GIT_TAG        v3.1.2   # 47f688ed6bb637ab8b7f4b36864734b2b1f69b6b
)
FetchContent_MakeAvailable(libASPL)

# ---- the HAL plug-in bundle ----
set(DRIVER_NAME "WingSimulator")
set(DRIVER_VERSION "0.1.0")
set(DRIVER_IDENTIFIER "com.wingsimulator.driver")
set(DRIVER_UID "5C6B9E21-7A0F-4E7B-9A3D-57494E475349")  # factory UUID
set(DRIVER_ENTRYPOINT "WingSimulatorEntryPoint")

add_library(${DRIVER_NAME} MODULE
  src/Driver.cpp
  src/ShmLink.cpp
)
target_include_directories(${DRIVER_NAME} PRIVATE include src)
# -Wno-reorder-init-list: libASPL's own Stream.hpp trips it (harmless).
target_compile_options(${DRIVER_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter
  -Wno-reorder-init-list)
target_link_libraries(${DRIVER_NAME} PRIVATE libASPL
  "-framework CoreFoundation" "-framework CoreAudio")
set_target_properties(${DRIVER_NAME} PROPERTIES
  OUTPUT_NAME "${DRIVER_NAME}"
  BUNDLE TRUE
  BUNDLE_EXTENSION "driver"
  PREFIX ""
  SUFFIX ""
  MACOSX_BUNDLE ON
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
  MACOSX_BUNDLE_BUNDLE_NAME "${DRIVER_NAME}"
  MACOSX_BUNDLE_BUNDLE_VERSION "${DRIVER_VERSION}"
  MACOSX_BUNDLE_SHORT_VERSION_STRING "${DRIVER_VERSION}"
  MACOSX_BUNDLE_GUI_IDENTIFIER "${DRIVER_IDENTIFIER}"
  MACOSX_BUNDLE_COPYRIGHT "Wing Simulator project"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
add_custom_command(TARGET ${DRIVER_NAME} POST_BUILD
  COMMAND codesign --force --sign "${CODESIGN_ID}" "$<TARGET_BUNDLE_DIR:${DRIVER_NAME}>"
  COMMENT "codesign ${DRIVER_NAME}.driver (${CODESIGN_ID})"
  VERBATIM)

# ---- host-side tools (no root needed) ----
# Shared-memory "daemon" test client: attaches like wing-sim will, applies gain.
add_executable(wing_shm_client tools/wing_shm_client.cpp)
target_include_directories(wing_shm_client PRIVATE include)

# Self test: drives ShmLink exactly like the HAL would and checks loopback and
# daemon paths against a real wing_shm_client process. Runs without install.
add_executable(wing_shm_selftest tools/wing_shm_selftest.cpp src/ShmLink.cpp)
target_include_directories(wing_shm_selftest PRIVATE include src)

# Post-install end-to-end probe through CoreAudio (talks to the installed device).
add_executable(wing_hal_probe tools/wing_hal_probe.cpp)
target_include_directories(wing_hal_probe PRIVATE include)
target_link_libraries(wing_hal_probe PRIVATE
  "-framework CoreAudio" "-framework CoreFoundation")

# In-process smoke test of the built bundle through the AudioServerPlugIn C
# interface with a fake host (no coreaudiod, no root).
add_executable(wing_driver_smoketest tools/wing_driver_smoketest.cpp)
target_include_directories(wing_driver_smoketest PRIVATE include)
target_link_libraries(wing_driver_smoketest PRIVATE
  "-framework CoreAudio" "-framework CoreFoundation")
add_dependencies(wing_driver_smoketest ${DRIVER_NAME})

enable_testing()
add_test(NAME shm_selftest
  COMMAND wing_shm_selftest --client $<TARGET_FILE:wing_shm_client>)
add_test(NAME driver_smoketest
  COMMAND wing_driver_smoketest $<TARGET_BUNDLE_DIR:${DRIVER_NAME}>)
