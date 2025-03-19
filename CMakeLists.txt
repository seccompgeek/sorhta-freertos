cmake_minimum_required(VERSION 3.10)

# Set the project name and language
project(freertos_s32g3 C ASM)

# Set toolchain
set(CMAKE_C_COMPILER aarch64-none-elf-gcc)
set(CMAKE_ASM_COMPILER aarch64-none-elf-gcc)
set(CMAKE_OBJCOPY aarch64-none-elf-objcopy)
set(CMAKE_SIZE aarch64-none-elf-size)

# Set the processor options - targeting 64-bit mode
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Compiler flags for 64-bit mode
set(COMMON_FLAGS "-march=armv8-a -mtune=cortex-a53")
#set(CMAKE_C_FLAGS "${COMMON_FLAGS} -O2 -g")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -Os -ffreestanding -fno-builtin")
set(CMAKE_EXE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/startup/s32g3_cortex_a53.ld -nostdlib -nostartfiles -fno-exceptions -fno-rtti -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CA53_64_BIT
)

# Source files
set(FREERTOS_SOURCES
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/tasks.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/list.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/queue.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/timers.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/event_groups.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CA53_64_BIT/port.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CA53_64_BIT/portASM.S
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

set(APP_SOURCES
    ${CMAKE_SOURCE_DIR}/app/main.c
    ${CMAKE_SOURCE_DIR}/startup/startup_s32g3_a53.c
    ${CMAKE_SOURCE_DIR}/startup/SystemInit.c
    ${CMAKE_SOURCE_DIR}/startup/linker_symbols.c
    #${CMAKE_SOURCE_DIR}/startup/vport_setup.c
)

# Add executable target
add_executable(${PROJECT_NAME}.elf ${FREERTOS_SOURCES} ${APP_SOURCES})

# Post-build commands to generate binary and hex files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf
)