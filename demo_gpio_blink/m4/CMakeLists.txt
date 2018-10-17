project("gpio_blink")

set(M4SDK_PATH "$ENV{M4SDK_PATH}")
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

cmake_minimum_required(VERSION 3.10,2)
enable_language(ASM)


set(COMMON_FLAGS "${COMMON_FLAGS} -DDEBUG")
set(COMMON_FLAGS "${COMMON_FLAGS} -D__STARTUP_CLEAR_BSS")
set(COMMON_FLAGS "${COMMON_FLAGS} -D__STARTUP_INITIALIZE_NONCACHEDATA")
set(COMMON_FLAGS "${COMMON_FLAGS} -g")
set(COMMON_FLAGS "${COMMON_FLAGS} -Wall")
set(COMMON_FLAGS "${COMMON_FLAGS} -fno-common")
set(COMMON_FLAGS "${COMMON_FLAGS} -ffunction-sections")
set(COMMON_FLAGS "${COMMON_FLAGS} -fdata-sections")
set(COMMON_FLAGS "${COMMON_FLAGS} -ffreestanding")
set(COMMON_FLAGS "${COMMON_FLAGS} -fno-builtin")
set(COMMON_FLAGS "${COMMON_FLAGS} -mthumb")
set(COMMON_FLAGS "${COMMON_FLAGS} -mapcs")
set(COMMON_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=cortex-m4")
set(COMMON_FLAGS "${COMMON_FLAGS} -mfloat-abi=hard")
set(COMMON_FLAGS "${COMMON_FLAGS} -mfpu=fpv4-sp-d16")
set(COMMON_FLAGS "${COMMON_FLAGS} -DCPU_MIMX8MQ6DVAJZ")
set(COMMON_FLAGS "${COMMON_FLAGS} -DFSL_RTOS_FREE_RTOS")
set(COMMON_FLAGS "${COMMON_FLAGS} -O0")
set(COMMON_FLAGS "${COMMON_FLAGS} -MMD")
set(COMMON_FLAGS "${COMMON_FLAGS} -MP")

set(CMAKE_ASM_FLAGS "${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -fno-rtti -fno-exceptions")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_FLAGS} --specs=nano.specs --specs=nosys.specs -Xlinker --gc-sections -Xlinker -static -Xlinker -z -Xlinker muldefs")


include_directories(${PROJECT_SOURCE_DIR})
include_directories(${M4SDK_PATH}/rtos/freertos/Source/portable/GCC/ARM_CM4F)
include_directories(${M4SDK_PATH}/CMSIS/Include)
include_directories(${M4SDK_PATH}/devices)
include_directories(${M4SDK_PATH}/rtos/freertos/Source/include)
include_directories(${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/platform/imx8mq_m4)
include_directories(${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6/drivers)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6/utilities/io)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6/utilities/str)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6/utilities/log)
include_directories(${M4SDK_PATH}/devices/MIMX8MQ6/utilities)
add_executable(${PROJECT_NAME}.elf 
  "${PROJECT_SOURCE_DIR}/fsl_iomuxc.h"
  "${PROJECT_SOURCE_DIR}/board.c"
  "${PROJECT_SOURCE_DIR}/board.h"
  "${PROJECT_SOURCE_DIR}/clock_config.c"
  "${PROJECT_SOURCE_DIR}/clock_config.h"
  "${PROJECT_SOURCE_DIR}/main.c"
  "${PROJECT_SOURCE_DIR}/FreeRTOSConfig.h"
  "${PROJECT_SOURCE_DIR}/pin_mux.c"
  "${PROJECT_SOURCE_DIR}/pin_mux.h"
  "${PROJECT_SOURCE_DIR}/rpmsg_config.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/platform/imx8mq_m4/rpmsg_platform.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/platform/imx8mq_m4/rpmsg_platform.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/environment/rpmsg_env_freertos.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_env.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_queue.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_queue.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_lite.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_ns.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/llist.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_default_config.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/virtio_ring.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/virtqueue.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_compiler.h"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/common/llist.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_lite.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_ns.c"
  "${M4SDK_PATH}/middleware/multicore/rpmsg_lite/lib/virtio/virtqueue.c"
  "${M4SDK_PATH}/rtos/freertos/FreeRTOS_for_MCUXpresso_SDK_v2.0.txt"
  "${M4SDK_PATH}/rtos/freertos/License/license.txt"
  "${M4SDK_PATH}/rtos/freertos/Source/include/croutine.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/event_groups.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/FreeRTOS.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/list.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/mpu_wrappers.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/portable.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/projdefs.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/queue.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/semphr.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/StackMacros.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/stdint.readme"
  "${M4SDK_PATH}/rtos/freertos/Source/include/task.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/timers.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/deprecated_definitions.h"
  "${M4SDK_PATH}/rtos/freertos/Source/include/freertos_tasks_c_additions.h"
  "${M4SDK_PATH}/rtos/freertos/Source/croutine.c"
  "${M4SDK_PATH}/rtos/freertos/Source/list.c"
  "${M4SDK_PATH}/rtos/freertos/Source/queue.c"
  "${M4SDK_PATH}/rtos/freertos/Source/readme.txt"
  "${M4SDK_PATH}/rtos/freertos/Source/tasks.c"
  "${M4SDK_PATH}/rtos/freertos/Source/timers.c"
  "${M4SDK_PATH}/rtos/freertos/Source/event_groups.c"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/readme.txt"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/GCC/ARM_CM4F/fsl_tickless_generic.h"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/GCC/ARM_CM4F/fsl_tickless_systick.c"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/GCC/ARM_CM4F/port.c"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/GCC/ARM_CM4F/portmacro.h"
  "${M4SDK_PATH}/rtos/freertos/Source/portable/MemMang/heap_4.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_clock.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_clock.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_common.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_common.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_rdc.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_rdc.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/io/fsl_io.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/io/fsl_io.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/log/fsl_log.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/log/fsl_log.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/str/fsl_str.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/str/fsl_str.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/fsl_debug_console.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/fsl_debug_console.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/fsl_debug_console_conf.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/system_MIMX8MQ6_cm4.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/system_MIMX8MQ6_cm4.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/gcc/startup_MIMX8MQ6_cm4.S"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_uart.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_uart.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_gpio.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_gpio.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/utilities/fsl_assert.c"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_mu.h"
  "${M4SDK_PATH}/devices/MIMX8MQ6/drivers/fsl_mu.c"
)
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${PROJECT_SOURCE_DIR}/MIMX8MQ6xxxJZ_cm4_ram.ld -static")


target_link_libraries(${PROJECT_NAME}.elf -Wl,--start-group)
target_link_libraries(${PROJECT_NAME}.elf debug m)
target_link_libraries(${PROJECT_NAME}.elf debug c)
target_link_libraries(${PROJECT_NAME}.elf debug gcc)
target_link_libraries(${PROJECT_NAME}.elf debug nosys)
target_link_libraries(${PROJECT_NAME}.elf optimized m)
target_link_libraries(${PROJECT_NAME}.elf optimized c)
target_link_libraries(${PROJECT_NAME}.elf optimized gcc)
target_link_libraries(${PROJECT_NAME}.elf optimized nosys)
target_link_libraries(${PROJECT_NAME}.elf optimized m)
target_link_libraries(${PROJECT_NAME}.elf optimized c)
target_link_libraries(${PROJECT_NAME}.elf optimized gcc)
target_link_libraries(${PROJECT_NAME}.elf optimized nosys)
target_link_libraries(${PROJECT_NAME}.elf optimized m)
target_link_libraries(${PROJECT_NAME}.elf optimized c)
target_link_libraries(${PROJECT_NAME}.elf optimized gcc)
target_link_libraries(${PROJECT_NAME}.elf optimized nosys)
target_link_libraries(${PROJECT_NAME}.elf -Wl,--end-group)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY}
  -Obinary ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_SIZE_UTIL}  ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf)
