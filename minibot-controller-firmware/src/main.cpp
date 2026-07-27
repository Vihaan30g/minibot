

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>  // esp_reset_reason() - for the diagnostics topic

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder.h"
#include "pid_controller.h"
#include "wheel.h"

/*
 * ============================================================
 * I2C Buses
 * ============================================================
 */

TwoWire LeftEncoderBus = TwoWire(0);
TwoWire RightEncoderBus = TwoWire(1);

/*
 * ============================================================
 * Motor Drivers
 * ============================================================
 */

MotorDriver leftMotor(
    LEFT_MOTOR_PWM_PIN,
    LEFT_MOTOR_DIR_PIN,
    LEFT_PWM_CHANNEL,
    LEFT_FORWARD,
    LEFT_REVERSE
);

MotorDriver rightMotor(
    RIGHT_MOTOR_PWM_PIN,
    RIGHT_MOTOR_DIR_PIN,
    RIGHT_PWM_CHANNEL,
    RIGHT_FORWARD,
    RIGHT_REVERSE
);

/*
 * ============================================================
 * Encoders
 * ============================================================
 */

Encoder leftEncoder(
    LeftEncoderBus,
    LEFT_ENCODER_SDA_PIN,
    LEFT_ENCODER_SCL_PIN,
    LEFT_COUNTS_PER_WHEEL_REV,
    LEFT_ENCODER_DIRECTION
);

Encoder rightEncoder(
    RightEncoderBus,
    RIGHT_ENCODER_SDA_PIN,
    RIGHT_ENCODER_SCL_PIN,
    RIGHT_COUNTS_PER_WHEEL_REV,
    RIGHT_ENCODER_DIRECTION
);

/*
 * ============================================================
 * PID Controllers
 * ============================================================
 */

PIDController rightPID(
    RIGHT_KP,
    RIGHT_KI,
    RIGHT_KD
);

PIDController leftPID(
    LEFT_KP,
    LEFT_KI,
    LEFT_KD
);

/*
 * ============================================================
 * Wheels
 * ============================================================
 */

Wheel leftWheel(
    leftMotor,
    leftEncoder,
    leftPID
);

Wheel rightWheel(
    rightMotor,
    rightEncoder,
    rightPID
);

/*
 * ============================================================
 * Watchdog state
 * ============================================================
 */

volatile uint32_t lastCommandTimeMs = 0;

/*
 * ============================================================
 * Loop health tracking (for diagnostics)
 * ============================================================
 */

uint32_t maxLoopDurationUs = 0;

/*
 * ============================================================
 * micro-ROS globals
 * ============================================================
 */

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmdSubscriber;
std_msgs__msg__Float32MultiArray cmdMsg;

rcl_publisher_t statePublisher;
sensor_msgs__msg__JointState stateMsg;

rcl_publisher_t diagPublisher;
std_msgs__msg__Int32MultiArray diagMsg;

rcl_publisher_t pwmPublisher;
std_msgs__msg__Int32MultiArray pwmMsg;

rcl_timer_t publishTimer;
rcl_timer_t diagTimer;

#define RCCHECK(fn)   { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

/*
 * ============================================================
 * Function Prototypes
 * ============================================================
 */

void errorLoop();
void controlLoopUpdate(double dt);
void publishWheelState();
void publishWheelPwm();
void publishDiagnostics();
void cmdCallback(const void* msgin);
void publishTimerCallback(rcl_timer_t* timer, int64_t lastCallTime);
void diagTimerCallback(rcl_timer_t* timer, int64_t lastCallTime);
void setupMessages();

/*
 * ============================================================
 * Error loop - blinks the onboard LED forever if micro-ROS setup fails
 * ============================================================
 */

void errorLoop()
{
    pinMode(STATUS_LED_PIN, OUTPUT);

    while (true)
    {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(150);
    }
}

/*
 * ============================================================
 * Control loop - called once per CONTROL_PERIOD from loop()
 * ============================================================
 */

void controlLoopUpdate(double dt)
{
    const uint32_t nowMs = millis();

    const bool commandStale =
        (nowMs - lastCommandTimeMs) > COMMAND_TIMEOUT_MS;

    if (commandStale)
    {
        // No fresh /wheel_cmd_vel within the timeout - force stop rather
        // than trusting the last command forever.
        leftWheel.stop();
        rightWheel.stop();
    }

    leftWheel.update(dt);
    rightWheel.update(dt);
}

/*
 * ============================================================
 * Publish wheel state - called from the micro-ROS publish timer
 * ============================================================
 */

void publishWheelState()
{
    stateMsg.position.data[0] = leftWheel.getCurrentPosition();
    stateMsg.position.data[1] = rightWheel.getCurrentPosition();

    stateMsg.velocity.data[0] = leftWheel.getCurrentVelocity();
    stateMsg.velocity.data[1] = rightWheel.getCurrentVelocity();

    RCSOFTCHECK(rcl_publish(&statePublisher, &stateMsg, NULL));
}

/*
 * ============================================================
 * Publish the actual signed PWM duty currently being sent to each motor
 * (-255..255, matches MOTOR_PWM_MIN/MAX). Lets you directly see on the
 * Jetson what the PID is commanding, alongside target vs measured
 * velocity from /wheel_states - very useful while tuning KP/KI/KD.
 * ============================================================ */

void publishWheelPwm()
{
    pwmMsg.data.data[0] = leftWheel.getCurrentPwm();
    pwmMsg.data.data[1] = rightWheel.getCurrentPwm();

    RCSOFTCHECK(rcl_publish(&pwmPublisher, &pwmMsg, NULL));
}

/*
 * ============================================================
 * Publish diagnostics - reset reason + free heap + worst-case loop time.
 *
 * data[0] = esp_reset_reason() code. Key values to watch for:
 *   1 = POWERON   (normal power-up, expected)
 *   3 = SW        (deliberate software reset, expected if you call one)
 *   4 = PANIC     (crash - check for null derefs, stack overflow, etc.)
 *   5 = INT_WDT   (interrupt watchdog - something blocked interrupts too long)
 *   6 = TASK_WDT  (task watchdog - a task, e.g. idle, was starved of CPU -
 *                  this is the one a busy-loop with no yield()/delay() causes)
 *   9 = BROWNOUT  (power supply sagged - check your buck converter/wiring)
 * data[1] = free heap, in KB
 * data[2] = worst-case single loop() iteration duration seen since boot, in us
 * ============================================================ */

void publishDiagnostics()
{
    diagMsg.data.data[0] = (int32_t)esp_reset_reason();
    diagMsg.data.data[1] = (int32_t)(ESP.getFreeHeap() / 1024);
    diagMsg.data.data[2] = (int32_t)maxLoopDurationUs;

    RCSOFTCHECK(rcl_publish(&diagPublisher, &diagMsg, NULL));
}

/*
 * ============================================================
 * micro-ROS callbacks
 * ============================================================
 */

void cmdCallback(const void* msgin)
{
    const std_msgs__msg__Float32MultiArray* msg =
        (const std_msgs__msg__Float32MultiArray*)msgin;

    if (msg->data.size >= 2)
    {
        leftWheel.setTargetVelocity(msg->data.data[0]);
        rightWheel.setTargetVelocity(msg->data.data[1]);
        lastCommandTimeMs = millis();
    }
}

void publishTimerCallback(rcl_timer_t* timer, int64_t lastCallTime)
{
    (void)lastCallTime;
    if (timer != NULL)
    {
        publishWheelState();
        publishWheelPwm();
    }
}

void diagTimerCallback(rcl_timer_t* timer, int64_t lastCallTime)
{
    (void)lastCallTime;
    if (timer != NULL)
    {
        publishDiagnostics();
    }
}

/*
 * ============================================================
 * Message pre-allocation
 * ============================================================
 */

void setupMessages()
{
    // --- Command message: Float32MultiArray[2] ---
    cmdMsg.data.data = (float*)malloc(2 * sizeof(float));
    cmdMsg.data.size = 0;
    cmdMsg.data.capacity = 2;

    // --- State message: JointState with 2 named joints ---
    stateMsg.name.data =
        (rosidl_runtime_c__String*)malloc(2 * sizeof(rosidl_runtime_c__String));
    stateMsg.name.size = 2;
    stateMsg.name.capacity = 2;

    const char* leftName  = "left_wheel_joint";
    const char* rightName = "right_wheel_joint";

    stateMsg.name.data[0].data = (char*)malloc(strlen(leftName) + 1);
    strcpy(stateMsg.name.data[0].data, leftName);
    stateMsg.name.data[0].size = strlen(leftName);
    stateMsg.name.data[0].capacity = strlen(leftName) + 1;

    stateMsg.name.data[1].data = (char*)malloc(strlen(rightName) + 1);
    strcpy(stateMsg.name.data[1].data, rightName);
    stateMsg.name.data[1].size = strlen(rightName);
    stateMsg.name.data[1].capacity = strlen(rightName) + 1;

    stateMsg.position.data = (double*)malloc(2 * sizeof(double));
    stateMsg.position.size = 2;
    stateMsg.position.capacity = 2;

    stateMsg.velocity.data = (double*)malloc(2 * sizeof(double));
    stateMsg.velocity.size = 2;
    stateMsg.velocity.capacity = 2;

    stateMsg.effort.data = NULL;
    stateMsg.effort.size = 0;
    stateMsg.effort.capacity = 0;

    // --- Diagnostics message: Int32MultiArray[3] ---
    diagMsg.data.data = (int32_t*)malloc(3 * sizeof(int32_t));
    diagMsg.data.size = 3;
    diagMsg.data.capacity = 3;

    // --- PWM message: Int32MultiArray[2] ---
    pwmMsg.data.data = (int32_t*)malloc(2 * sizeof(int32_t));
    pwmMsg.data.size = 2;
    pwmMsg.data.capacity = 2;
}

/*
 * ============================================================
 * Setup
 * ============================================================
 */

void setup()
{
    set_microros_serial_transports(Serial);
    Serial.begin(UROS_SERIAL_BAUD);
    delay(2000);  // give the agent side time to be ready after boot

    leftMotor.begin();
    rightMotor.begin();

    leftEncoder.begin();
    rightEncoder.begin();

    leftWheel.begin();
    rightWheel.begin();

    setupMessages();

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, UROS_NODE_NAME, UROS_NODE_NAMESPACE, &support));

    RCCHECK(rclc_subscription_init_default(
        &cmdSubscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        UROS_CMD_TOPIC));

    RCCHECK(rclc_publisher_init_best_effort(
        &statePublisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        UROS_STATE_TOPIC));

    RCCHECK(rclc_publisher_init_best_effort(
        &diagPublisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        UROS_DIAG_TOPIC));

    RCCHECK(rclc_publisher_init_best_effort(
        &pwmPublisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        UROS_PWM_TOPIC));

    RCCHECK(rclc_timer_init_default(
        &publishTimer, &support, RCL_MS_TO_NS(UROS_PUBLISH_PERIOD_MS), publishTimerCallback));

    RCCHECK(rclc_timer_init_default(
        &diagTimer, &support, RCL_MS_TO_NS(UROS_DIAG_PERIOD_MS), diagTimerCallback));

    // executor handles: 1 subscription + 2 timers
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmdSubscriber, &cmdMsg, &cmdCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &publishTimer));
    RCCHECK(rclc_executor_add_timer(&executor, &diagTimer));

    lastCommandTimeMs = millis();  // don't immediately trip the watchdog at boot
}

/*
 * ============================================================
 * Main Loop
 * ============================================================
 */

void loop()
{
    const uint32_t loopStartUs = micros();

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

    static uint32_t lastControlTime = micros();

    const uint32_t now = micros();

    while ((now - lastControlTime) >= CONTROL_PERIOD_US)
    {
        lastControlTime += CONTROL_PERIOD_US;

        controlLoopUpdate(CONTROL_PERIOD);
    }

    const uint32_t loopDurationUs = micros() - loopStartUs;
    if (loopDurationUs > maxLoopDurationUs)
    {
        maxLoopDurationUs = loopDurationUs;
    }

    // CRITICAL: without this, loop() never blocks or yields, which starves
    // the FreeRTOS idle task and trips the Task Watchdog Timer (~5s default)
    // -> the whole board reboots in a loop. This one line is the fix for
    // the "wheel_states only arrives every few seconds" symptom.
    yield();
}


