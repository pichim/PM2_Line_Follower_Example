#include <mbed.h>
#include <math.h>

#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/EncoderCounter.h"
#include "pm2_drivers/DCMotor.h"
#include "pm2_drivers/SensorBar.h"
#include "pm2_drivers/PESBoardPinMap.h"
#include "eigen/Dense.h"

#define M_PI 3.14159265358979323846  // number pi

// logical variable main task
bool do_execute_main_task = false; 

// user button on Nucleo board
Timer user_button_timer;            
DebounceIn user_button(USER_BUTTON);
// Function that triggers main task execution   
void user_button_pressed_fcn();    

// controller functions
float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle);
//float vel_cntrl_v1_fcn(const float& vel_max, const float& vel_min, const float& ang_max, const float& angle);
float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);

int main()
{
    // states and actual state for the state machine, the machine will have 3 states:
    // initial - to enable all systems
    // follow - to follow the line
    // sleep - to wait for the signal from the environment (e.g. line detection)
    enum RobotState {
        INITIAL,
        FOLLOW,
        SLEEP,
    } robot_state = RobotState::INITIAL;

    // attach button fall function to user button object
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 20;   // define main task period time in ms
    Timer main_task_timer;  

    // led on nucleo board
    DigitalOut user_led(USER_LED); 

    // robot kinematics
    const float r_wheel = 0.0358f / 2.0f; // wheel radius
    const float L_wheel = 0.143f;         // distance from wheel to wheel
    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    //Eigen::Matrix2f Crobot2wheel; // transform robot to wheel
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / L_wheel, -r_wheel / L_wheel;
    //Crobot2wheel << 1.0f / r_wheel,  L_wheel / (2.0f * r_wheel),
    //                1.0f / r_wheel, -L_wheel / (2.0f * r_wheel);
    Eigen::Vector2f robot_coord;  // contains v and w (robot translational and rotational velocities)
    Eigen::Vector2f wheel_speed;  // w1 w2 (wheel speed)
    robot_coord.setZero();
    wheel_speed.setZero();

    // sensor data evalution
    const float bar_dist = 0.1175f; // distance from bar to wheel axis 

    // line following sensor object definition
    I2C i2c(PB_9, PB_8);
    SensorBar sensor_bar(i2c, bar_dist);

    // digital out object for enabling motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // in the robot there will be used 78:1 Metal Gearmotor 20Dx44L mm 12V CB
    // define variables and create DC motor objects
    const float voltage_max = 12.0f;
    const float gear_ratio = 78.125f; 
    const float kn = 180.0f / 12.0f; //motor constant rpm / V
    const float velocity_max = kn * voltage_max / 60.0f; // Max velocity that can be reached
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    motor_M1.setMaxVelocity(velocity_max); 
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    motor_M2.setMaxVelocity(velocity_max);

    // condition for state machine that will stop the robot 1 seconds after leaving the line (CAN BE CHANGED)
    bool move = false;
    const static float stop_time = 1.0f; //seconds
    const static int stop_time_iteration = stop_time * 1000 / main_task_period_ms;
    static int i = stop_time_iteration + 1;

    // timer to measure task execution time
    main_task_timer.start();

    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) {

            static float sensor_bar_avgAngleRad = 0.0f;

            // line detection checking
            if (sensor_bar.isAnyLedActive()) {
                sensor_bar_avgAngleRad = sensor_bar.getAvgAngleRad();
                i = 0;
            } else {
                i += 1;
            }

            // robot stop condition checking
            if (i > stop_time_iteration) {
                move = false;
            } else {
                move = true;
            }

            // robot angular velocity evaluation
            const static float Kp = 3.0f; // by making this const static it will not be overwritten and only initiliazed once
            const static float Kp_nl = 17.0f;
            robot_coord(1) = ang_cntrl_fcn(Kp, Kp_nl, sensor_bar_avgAngleRad);
            
            // nonlinear controllers version 2 (one wheel always at full speed controller)
            const static float wheel_speed_max = 23.0f * voltage_max * kn / (60.0f * 2.0f * M_PI); // TODO check the wheel speed on map
            const static float b = L_wheel / (2.0f * r_wheel);
            robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max, b, robot_coord(1), Cwheel2robot);

            // transform robot coordinates to wheel speed
            wheel_speed = Cwheel2robot.inverse() * robot_coord;

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL:
                    enable_motors = 1;
                    if (move == true) {
                        robot_state = RobotState::FOLLOW;
                    } else {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::FOLLOW:
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

                    if (move == false) {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::SLEEP:
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);

                    if (move == true) {
                        robot_state = RobotState::FOLLOW;
                    }
                    break;
                default:
                    break; // do nothing
            }
        }
        // toggling user-led
        user_led = !user_led;

        // printing parameters
        printf("%f, %f, %f\r\n", motor_M1.getVelocity(), motor_M2.getVelocity(), sensor_bar.getAvgAngleRad() * 180.0f / M_PI);

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
}

float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle)
{
    static float retval = 0.0f;
    if (angle > 0) {
        retval = Kp * angle + Kp_nl * angle * angle;
    } else if (angle <= 0) {
        retval = Kp * angle - Kp_nl * angle * angle;
    }
    return retval;
}

/*
float vel_cntrl_v1_fcn(const float& vel_max, const float& vel_min, const float& ang_max, const float& angle)
{
    const static float gain = (vel_min - vel_max) / ang_max;
    const static float offset = vel_max;
    return gain * fabs(angle) + offset;
}
*/

float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot)
{
    static Eigen::Matrix<float, 2, 2> _wheel_speed;
    static Eigen::Matrix<float, 2, 2> _robot_coord;
    if (robot_omega > 0) {
        _wheel_speed(0) = wheel_speed_max;
        _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega;
    } else {
        _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
        _wheel_speed(1) = wheel_speed_max;
    }
    _robot_coord = Cwheel2robot * _wheel_speed;

    return _robot_coord(0);
}
