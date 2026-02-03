cmake_minimum_required(VERSION 3.20)
project(transiver C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(MCPU "-mcpu=cortex-m3")
set(MTHUMB "-mthumb")

# sources
add_executable(${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/src/main.c
  ${CMAKE_SOURCE_DIR}/src/init.c
  ${CMAKE_SOURCE_DIR}/src/uart.c
  ${CMAKE_SOURCE_DIR}/params/param.c
  ${CMAKE_SOURCE_DIR}/cmsis/SAM3X/Device/Source/system_SAM3XA.c
  ${CMAKE_SOURCE_DIR}/startup/startup_sam3x8e.c
  ${CMAKE_SOURCE_DIR}/src/sin_lut_1024.c
)

# ---- includes/defines/options AFTER target exists ----
target_compile_definitions(${PROJECT_NAME} PRIVATE __SAM3X8E__)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_SOURCE_DIR}/src/include
  ${CMAKE_SOURCE_DIR}/cmsis
  ${CMAKE_SOURCE_DIR}/cmsis/Core/Include
  ${CMAKE_SOURCE_DIR}/cmsis/SAM3X/Device/Include
  ${CMAKE_SOURCE_DIR}/cmsis/SAM3X/Device/Include/component
  ${CMAKE_SOURCE_DIR}/cmsis/SAM3X/Device/Include/instance
  ${CMAKE_SOURCE_DIR}/cmsis/SAM3X/Device/Include/pio
)

target_compile_options(${PROJECT_NAME} PRIVATE
  ${MCPU} ${MTHUMB}
  -ffunction-sections -fdata-sections
  -fno-builtin
  -Wall -Wextra
  -Os
)

# linker script path as CMake-style (C:/...)
set(LD_SCRIPT "${CMAKE_SOURCE_DIR}/linker/sam3x8e_flash.ld")
file(TO_CMAKE_PATH "${LD_SCRIPT}" LD_SCRIPT_CMAKE)

target_link_options(${PROJECT_NAME} PRIVATE
  ${MCPU} ${MTHUMB}
  -Wl,--gc-sections
  "-Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map"
  "-T${LD_SCRIPT_CMAKE}"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}"
  SUFFIX ".elf"
)

find_program(OBJCOPY arm-none-eabi-objcopy REQUIRED)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
  COMMAND ${OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
  COMMENT "Generating ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
)

