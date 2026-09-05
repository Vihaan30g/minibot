#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*
 * ============================
 * Left Motor Pins
 * ============================
 */
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 25;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 26;

/*
 * ============================
 * Right Motor Pins
 * ============================
 */
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 27;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 14;

/*
 * ============================
 * PWM Channels
 * ============================
 */
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

/*
 * ============================
 * PWM Configuration
 * ============================
 */
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint16_t MAX_PWM = 255;

/*
 * ============================
 * Motor Directions
 *
 * If a motor rotates opposite
 * to the expected direction,
 * simply swap HIGH and LOW.
 * ============================
 */
constexpr bool LEFT_FORWARD = HIGH;
constexpr bool LEFT_REVERSE = LOW;

constexpr bool RIGHT_FORWARD = LOW;
constexpr bool RIGHT_REVERSE = HIGH;


/* ============================================================
 * Encoder I2C
 * (renamed to match main.cpp's usage - this was the mismatch
 * causing the compile failure)
 * ============================================================ */

constexpr uint8_t LEFT_ENCODER_SDA_PIN = 21;
constexpr uint8_t LEFT_ENCODER_SCL_PIN = 22;

constexpr uint8_t RIGHT_ENCODER_SDA_PIN = 32;
constexpr uint8_t RIGHT_ENCODER_SCL_PIN = 33;

constexpr uint8_t AS5600_ADDRESS = 0x36;

constexpr uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0C;

constexpr uint16_t AS5600_RESOLUTION = 4096;

constexpr uint16_t AS5600_HALF_RESOLUTION =
    AS5600_RESOLUTION / 2;


/* ============================================================
 * Calibration
 * Counts accumulated for ONE wheel revolution
 * ============================================================ */

constexpr double LEFT_COUNTS_PER_WHEEL_REV = 997398.166666667;

constexpr double RIGHT_COUNTS_PER_WHEEL_REV = 994691.0;

/* ============================================================
 * Encoder direction correction
 *
 * The two motors face each other (mirrored mounting), so a rotation
 * that drives the robot straight forward spins the two motor shafts in
 * OPPOSITE handedness as measured by each motor's own encoder - this is
 * expected physics, not a wiring fault.
 *
 * Determined empirically: after driving forward, left position came out
 * positive (correct) and right came out negative (needs flipping) - so
 * RIGHT gets -1. If you ever re-mount or swap a motor, redo this check:
 * drive forward, watch both signs, flip whichever one reads negative.
 * ============================================================ */

constexpr int8_t LEFT_ENCODER_DIRECTION  = 1;
constexpr int8_t RIGHT_ENCODER_DIRECTION = -1;


/* ============================================================
 * Control Loop
 * ============================================================ */

constexpr double CONTROL_FREQUENCY = 1000.0;      // Hz

constexpr double CONTROL_PERIOD =
    1.0 / CONTROL_FREQUENCY;                      // seconds

constexpr uint32_t CONTROL_PERIOD_US =
    1000000UL / CONTROL_FREQUENCY;               // microseconds


/* ============================================================
 * PID Output Limits
 * ============================================================ */

constexpr double MOTOR_PWM_MIN = -255.0;

constexpr double MOTOR_PWM_MAX = 255.0;


/* ============================================================
 * PID Integral Limits
 * ============================================================ */

constexpr double PID_INTEGRAL_MIN = -150.0;

constexpr double PID_INTEGRAL_MAX = 150.0;


/* ============================================================
 * Default PID Gains
 *
 * These were left at 0 as placeholders and NEVER updated - with all
 * three at zero, PIDController::compute() always returns 0 regardless
 * of setpoint or measurement, so the motor never receives any PWM.
 * This is the most likely reason the robot doesn't move.
 *
 * KP below is a conservative starting guess only, NOT a tuned value -
 * put the robot up on blocks (wheels off the ground) and tune from here:
 * increase KP until the wheel tracks a step change in target velocity
 * reasonably quickly without oscillating, then add a small KD if there's
 * overshoot, then a small KI only if steady-state error remains.
 * ============================================================ */

constexpr double LEFT_KP = 15.0;

constexpr double LEFT_KI = 0.0;

constexpr double LEFT_KD = 0.0;



constexpr double RIGHT_KP = 15.0;

constexpr double RIGHT_KI = 0.0;

constexpr double RIGHT_KD = 0.0;


/* ============================================================
 * Safety
 * ============================================================ */

constexpr uint32_t COMMAND_TIMEOUT_MS = 500;


/* ============================================================
 * micro-ROS
 * ============================================================ */

constexpr uint32_t UROS_SERIAL_BAUD = 921600;

constexpr char UROS_NODE_NAME[]      = "esp32_diff_drive_node";
constexpr char UROS_NODE_NAMESPACE[] = "";

constexpr char UROS_CMD_TOPIC[]   = "wheel_cmd_vel";  // sub: std_msgs/Float32MultiArray [left_rad_s, right_rad_s]
constexpr char UROS_STATE_TOPIC[] = "wheel_states";   // pub: sensor_msgs/JointState
constexpr char UROS_PWM_TOPIC[]   = "wheel_pwm";      // pub: std_msgs/Int32MultiArray [left_pwm, right_pwm], published alongside wheel_states

constexpr double UROS_PUBLISH_FREQUENCY = 50.0;       // Hz
constexpr uint32_t UROS_PUBLISH_PERIOD_MS =
    (uint32_t)(1000.0 / UROS_PUBLISH_FREQUENCY);

constexpr uint8_t STATUS_LED_PIN = 2;  // onboard LED, used for the micro-ROS error loop

/* ============================================================
 * Diagnostics
 * A dedicated low-rate topic reporting reset reason and loop health, so
 * future debugging doesn't depend on guessing from symptoms alone -
 * `ros2 topic echo /esp32_diagnostics` tells you directly if the board
 * is rebooting and why.
 * ============================================================ */

constexpr char UROS_DIAG_TOPIC[] = "esp32_diagnostics";  // pub: std_msgs/Int32MultiArray
constexpr double UROS_DIAG_FREQUENCY = 1.0;               // Hz
constexpr uint32_t UROS_DIAG_PERIOD_MS =
    (uint32_t)(1000.0 / UROS_DIAG_FREQUENCY);

#endif








































