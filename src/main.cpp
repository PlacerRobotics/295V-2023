#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
ASSET(path_txt);
ASSET(path1_txt);
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);
ASSET(path5_txt);

// Inertial Sensor on port 11
pros::Imu imu(12);

int auton = 0;

// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain
lemlib::Drivetrain_t drivetrain {&leftMotors, &rightMotors, 10, lemlib::Omniwheel::NEW_4, 600, 2};

// lateral motion controller
lemlib::ChassisController_t lateralController {10, 30, 1, 100, 3, 500, 20};

// angular motion controller
lemlib::ChassisController_t angularController {2, 10, 1, 100, 3, 500, 20};

// sensors for odometry
lemlib::OdomSensors_t sensors {nullptr, nullptr, &horizontal, nullptr, &imu};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


pros::Task loadCatapultTask{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 6000; 

        // normal shot
        catapultMotor.move_voltage(-12000); // 85
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(20);
            if(catapultRotation.get_angle() > pullbackAngle-1500){
                catapultMotor.move_voltage(-9000);
            }
        }

        catapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapultMotor.move_voltage(0);

        pros::delay(20);
    }
}
};
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void autonred1(){ // General Red Side Auton
    // chassis.turnTo(53, 53, 1000); // turn to the point (53, 53) with a timeout of 1000 ms
    // chassis.turnTo(-20, 32, 1500, true); // turn to the point (-20, 32) with the back of the robot facing the point, and a timeout of 1500 ms
    // chassis.turnTo(10, 0, 1000, false, 50); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50
    // chassis.moveTo(53, 53, 1000); // move to the point (53, 53) with a timeout of 1000 ms
    // chassis.moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

    // In order for the robot to read the file, we need to put it on a micro SD card. Simply drag the file onto the SD card and it will be copied over. You can then insert the SD card into the robot and it will be able to read the file.

    // file name: path.txt
    // timeout: 2000 ms
    // lookahead distance: 15 inches
    // chassis.follow("lemlib/path.txt", 2000, 15);
    // chassis.follow(path_txt, 2000, 15);
    chassis.follow(path1_txt, 2000, 15);
    intakeMotor.move_voltage(-12000);
    intakePiston1.set_value(1);
    intakePiston2.set_value(1);
    chassis.follow(path2_txt, 2000, 15);
    intakeMotor.move_voltage(12000);
    intakePiston1.set_value(0);
    intakePiston2.set_value(0);
    chassis.follow(path3_txt, 2000, 15);
    intakeMotor.move_voltage(-12000);
    intakePiston1.set_value(1);
    intakePiston2.set_value(1);
    // imu.set_heading(0);
    // while(imu.get_heading() <= 180){
    //     leftMotors.move_voltage(6000);
    //     rightMotors.move_voltage(-6000);
    // }
    chassis.turnTo(13.651257848803102, -0.6833656034310727, 500);
    intakePiston1.set_value(1);
    intakePiston2.set_value(1);
    chassis.follow(path4_txt, 2000, 15);
    intakePiston1.set_value(0);
    intakePiston2.set_value(0);
    intakeMotor.move_voltage(-12000);
    chassis.follow(path5_txt, 2000, 15);
    intakeMotor.move_velocity(12000);

    // // follow the next path, but with the robot going backwards
    // chassis.follow("path.txt", 2000, 15, true);
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

void initialize() {
    pros::lcd::initialize();
    lemlib::Logger::initialize();
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0

    // print odom values to the brain
    pros::Task screenTask(screen);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/*
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
*/

void autonomous() {
    if(auton == 1){
        autonred1();
    }
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() { 
    // chassis.moveTo(20, 15, 90, 4000); 
    allMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    while (true) {
        pros::screen::set_pen(COLOR_WHITE);
        pros::lcd::print(0, 0, catapultRotation.get_angle());
        chassis.standard();
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            intakeMotor.move_voltage(4500);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor.move_voltage(-12000);
        }
        else{
            intakeMotor.move_voltage(0);
        }
       if(cataDistance.get() <= 7){
        loadCatapultTask.notify();
       } 
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        wing1.set_value(1);
        wing2.set_value(1);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        wing1.set_value(0);
        wing2.set_value(0);
       }
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        intakePiston1.set_value(1);
        intakePiston2.set_value(1);
       } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        intakePiston1.set_value(0);
        intakePiston2.set_value(0);
       }
    }
}
