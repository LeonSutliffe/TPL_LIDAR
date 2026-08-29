#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_usbcdc_logging.h"
#include "esp_usbcdc_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

#include "pins.h"
#include "mks_servo42d.h"
#include "tilt_axis.h"

/*
 * Motor control node for the tilt axis, per the project brief:
 *   - owns the RS485 link to the MKS SERVO42D driver
 *   - executes move/home commands, in either step-and-stare (discrete moves)
 *     or continuous-sweep mode -- same tilt/cmd_position_deg entry point
 *     either way, selected via tilt/cmd_sweep_rate_deg_s (0 = step-and-stare)
 *   - publishes sensor_msgs/JointState for the tilt joint
 *   - publishes a simple idle/homing/moving/sweeping/settling/settled status topic
 *   - homing is triggered once per session by the PC, not autonomously
 *
 * Transport: micro-ROS custom transport over the ESP32-S2's native USB
 * (GPIO19/20), exposing two USB-CDC ACM interfaces -- one for the micro-ROS
 * agent link, one for log output -- so RS485 (UART1, separate pins) and
 * flashing/monitor (USB-Serial-JTAG, GPIO43/44) stay untouched. This
 * structure (rmw_uros_set_custom_transport + esp_usbcdc_open/close/write/read)
 * is taken directly from micro_ros_espidf_component's own verified
 * examples/int32_publisher_custom_transport_usbcdc, not reconstructed from
 * memory, so it should build as-is against the vendored jazzy branch. See
 * README.md for the matching agent invocation and sdkconfig requirements.
 */

#define NODE_NAME CONFIG_IDF_TARGET
#define CONTROL_TIMER_PERIOD_MS 100

#define TAG_MAIN "MAIN"
#define TAG_TASK "MICRO_ROS"

#define RCCHECK(fn) do { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG_TASK, "Failed status on line %d: %d. Aborting.", __LINE__, (int)rc); \
        vTaskDelete(NULL); \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG_TASK, "Failed status on line %d: %d. Continuing.", __LINE__, (int)rc); \
    } \
} while (0)

static rcl_publisher_t joint_state_pub;
static rcl_publisher_t status_pub;
static rcl_subscription_t target_sub;
static rcl_subscription_t home_sub;
static rcl_subscription_t sweep_rate_sub;

static sensor_msgs__msg__JointState joint_state_msg;
static std_msgs__msg__String status_msg;
static std_msgs__msg__Float32 target_msg;
static std_msgs__msg__Bool home_msg;
static std_msgs__msg__Float32 sweep_rate_msg;

/* Statically allocated JointState backing storage (single joint) -- avoids
 * depending on the micro_ros_utilities dynamic-allocation helpers. */
static rosidl_runtime_c__String joint_name_storage[1];
static double joint_position_storage[1];

static mks_handle_t mks;
static tilt_axis_t tilt;

static void target_sub_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    tilt_axis_move_to_deg(&tilt, msg->data);
}

static void home_sub_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        tilt_axis_start_homing(&tilt);
    }
}

static void sweep_rate_sub_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    tilt_axis_set_sweep_rate(&tilt, msg->data);
    ESP_LOGI(TAG_MAIN, "sweep rate set to %.3f deg/s (%s mode)", (double)msg->data,
             msg->data > 0.0f ? "continuous sweep" : "step-and-stare");
}

static void publish_joint_state(void)
{
    joint_position_storage[0] = tilt_axis_get_position_deg(&tilt) * (float)M_PI / 180.0f;
    RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

static void publish_status(void)
{
    rosidl_runtime_c__String__assign(&status_msg.data, tilt_axis_get_status_string(&tilt));
    RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
}

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) {
        return;
    }
    tilt_axis_status_t prev_status = tilt.status;
    tilt_axis_update(&tilt);
    publish_joint_state();
    if (tilt.status != prev_status) {
        publish_status();
    }
}

static void init_joint_state_msg(void)
{
    rosidl_runtime_c__String__init(&joint_name_storage[0]);
    rosidl_runtime_c__String__assign(&joint_name_storage[0], "tilt_joint");

    joint_state_msg.name.data = joint_name_storage;
    joint_state_msg.name.size = 1;
    joint_state_msg.name.capacity = 1;

    joint_state_msg.position.data = joint_position_storage;
    joint_state_msg.position.size = 1;
    joint_state_msg.position.capacity = 1;

    joint_state_msg.velocity.data = NULL;
    joint_state_msg.velocity.size = 0;
    joint_state_msg.velocity.capacity = 0;

    joint_state_msg.effort.data = NULL;
    joint_state_msg.effort.size = 0;
    joint_state_msg.effort.capacity = 0;

    rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_MAIN, "initializing MKS RS485 link");
    esp_err_t mks_init_err = mks_init(&mks, RS485_UART_PORT, RS485_TX_PIN, RS485_RX_PIN, RS485_RTS_PIN,
                                       38400 /* baud -- verify against MKS driver's configured baud rate */,
                                       1 /* slave address -- verify against MKS driver's DIP/config */);
    /* Non-fatal: the MKS driver may not be connected/powered yet (e.g. bench
     * testing the ESP32 alone). A dead RS485 link shouldn't crash the whole
     * node -- USB/ROS still need to come up so the failure is visible over
     * tilt/status instead of silently reboot-looping. tilt_axis_update()
     * already treats unanswered mks_* calls as "not ready" rather than
     * assuming init succeeded. */
    if (mks_init_err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "mks_init() failed: %d -- MKS driver not responding, continuing without it",
                 (int)mks_init_err);
    }
    tilt_axis_init(&tilt, &mks);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support = {0};
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "tilt_axis_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &joint_state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "tilt/joint_state"));
    RCCHECK(rclc_publisher_init_default(
        &status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "tilt/status"));
    RCCHECK(rclc_subscription_init_default(
        &target_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "tilt/cmd_position_deg"));
    RCCHECK(rclc_subscription_init_default(
        &home_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "tilt/cmd_home"));
    RCCHECK(rclc_subscription_init_default(
        &sweep_rate_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "tilt/cmd_sweep_rate_deg_s"));

    init_joint_state_msg();
    rosidl_runtime_c__String__init(&status_msg.data);

    rcl_timer_t control_timer;
    RCCHECK(rclc_timer_init_default2(&control_timer, &support, RCL_MS_TO_NS(CONTROL_TIMER_PERIOD_MS),
                                      control_timer_callback, true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &target_sub, &target_msg,
                                            &target_sub_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &home_sub, &home_msg,
                                            &home_sub_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sweep_rate_sub, &sweep_rate_msg,
                                            &sweep_rate_sub_callback, ON_NEW_DATA));

    ESP_LOGI(TAG_MAIN, "tilt_axis_node ready");

    while (true) {
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
            ESP_LOGW(TAG_TASK, "rclc_executor_spin_some() failed: %d", (int)spin_ret);
        }
        usleep(10000);
    }

    vTaskDelete(NULL);
}

static tinyusb_cdcacm_itf_t cdc_port = TINYUSB_CDC_ACM_0; /* micro-ROS agent interface */

void app_main(void)
{
#if (CONFIG_TINYUSB_CDC_COUNT >= 2)
    if (esp_usbcdc_logging_init() == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "USB-CDC logging initialized");
    }
#endif

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_ret_t ret = rmw_uros_set_custom_transport(
        true,
        (void *)&cdc_port,
        esp_usbcdc_open,
        esp_usbcdc_close,
        esp_usbcdc_write,
        esp_usbcdc_read);

    if (ret != RMW_RET_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to set micro-ROS custom transport layer");
        return;
    }
#else
#error micro-ROS transport misconfigured -- expected RMW_UXRCE_TRANSPORT_CUSTOM (see README.md)
#endif

    TaskHandle_t task_handle = NULL;
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO, &task_handle);

    if (task_handle != NULL) {
        ESP_LOGI(TAG_MAIN, "micro-ROS task created");
    }
}
