/**
 * @file src/lemlib/pid.cpp
 * @author LemLib Team
 * @brief FAPID class member definitions
 * @version 0.4.5
 * @date 2023-01-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <math.h>
#include "lemlib/pid.hpp"
#include "lemlib/util.hpp"
#include "lemlib/logger.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"

// define static variables
std::string lemlib::FAPID::input = "FAPID";
pros::Task* lemlib::FAPID::logTask = nullptr;
pros::Mutex lemlib::FAPID::logMutex = pros::Mutex();

/**
 * @brief Construct a new FAPID
 *
 * @param kF feedfoward gain, multiplied by target and added to output. Set 0 if disabled
 * @param kA acceleration gain, limits the change in output. Set 0 if disabled
 * @param kP proportional gain, multiplied by error and added to output
 * @param kI integral gain, multiplied by total error and added to output
 * @param kD derivative gain, multiplied by change in error and added to output
 * @param name name of the FAPID. Used for logging
 */
lemlib::FAPID::FAPID(float kF, float kA, float kP, float kI, float kD, std::string name) {
    this->kF = kF;
    this->kA = kA;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->name = name;
}

/**
 * @brief Set gains
 *
 * @param kF feedfoward gain, multiplied by target and added to output. Set 0 if disabled
 * @param kA acceleration gain, limits the change in output. Set 0 if disabled
 * @param kP proportional gain, multiplied by error and added to output
 * @param kI integral gain, multiplied by total error and added to output
 * @param kD derivative gain, multiplied by change in error and added to output
 */
void lemlib::FAPID::setGains(float kF, float kA, float kP, float kI, float kD) {
    this->kF = kF;
    this->kA = kA;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

/**
 * @brief Set the exit conditions
 *
 * @param largeError
 * @param smallError
 * @param largeTime
 * @param smallTime
 * @param maxTime
 */
void lemlib::FAPID::setExit(float largeError, float smallError, int largeTime, int smallTime, int maxTime) {
    this->largeError = largeError;
    this->smallError = smallError;
    this->largeTime = largeTime;
    this->smallTime = smallTime;
    this->maxTime = maxTime;
}

/**
 * @brief Update the FAPID
 *
 * @param target the target value
 * @param position the current value
 * @param log whether to check the most recent terminal input for user input. Default is false because logging multiple
 * PIDs could slow down the program.
 * @return float - output
 */
float lemlib::FAPID::update(float target, float position, bool log) {
    // check most recent input if logging is enabled
    // this does not run by default because the mutexes could slow down the program
    // calculate output
    float error = target - position;
    float deltaError = error - prevError;
    float output = kF * target + kP * error + kI * totalError + kD * deltaError;
    if (kA != 0) output = lemlib::slew(output, prevOutput, kA);
    prevOutput = output;
    prevError = error;
    totalError += error;

    if (log) { Logger::logPid(name, output, error, kP * error, kD * totalError, kD * deltaError); }
    return output;
}

/**
 * @brief Reset the FAPID
 */
void lemlib::FAPID::reset() {
    prevError = 0;
    totalError = 0;
    prevOutput = 0;
}

/**
 * @brief Check if the FAPID has settled
 *
 * If the exit conditions have not been set, this function will always return false
 *
 * @return true - the FAPID has settled
 * @return false - the FAPID has not settled
 */
bool lemlib::FAPID::settled() {
    if (startTime == 0) { // if maxTime has not been set
        startTime = pros::c::millis();
        return false;
    } else { // check if the FAPID has settled
        if (pros::c::millis() - startTime > maxTime) return true; // maxTime has been exceeded
        if (std::fabs(prevError) < largeError) { // largeError within range
            if (!largeTimeCounter) largeTimeCounter = pros::c::millis(); // largeTimeCounter has not been set
            else if (pros::c::millis() - largeTimeCounter > largeTime) return true; // largeTime has been exceeded
        }
        if (std::fabs(prevError) < smallError) { // smallError within range
            if (!smallTimeCounter) smallTimeCounter = pros::c::millis(); // smallTimeCounter has not been set
            else if (pros::c::millis() - smallTimeCounter > smallTime) return true; // smallTime has been exceeded
        }
        // if none of the exit conditions have been met
        return false;
    }
}

/**
 * @brief Enable logging
 * the user can interact with the FAPID through the terminal
 * the user can access gains and other variables with the following format:
 * <name>.<variable> to get the value of the variable
 * <name>.<variable>_<value> to set the value of the variable
 * for example:z
 * pid.kP_0.5 will set the kP value to 0.5
 * list of variables thats value can be set:
 * kF, kA, kP, kI, kD
 * list of variables that can be accessed:
 * kF, kA, kP, kI, kD, totalError
 * list of functions that can be called:
 * reset()
 */
void lemlib::FAPID::init() {
    if (logTask != nullptr) {
        logTask = new pros::Task {[=] {
            while (true) {
                // get input
                std::cin >> input;
                pros::delay(20);
            }
        }};
    }
}

/**
 * @brief Log the FAPID
 */
void lemlib::FAPID::log() {
    // check if the input starts with the name of the FAPID
    // try to obtain the logging mutex
    if (logMutex.take(5)) {
        if (input.find(name) == 0) {
            // remove the name from the input
            input.erase(0, name.length() + 1);
            // check if the input is a function
            if (input == "reset()") {
                reset();
            } else if (input == "kF") {
                std::cout << kF << std::endl;
            } else if (input == "kA") {
                std::cout << kA << std::endl;
            } else if (input == "kP") {
                std::cout << kP << std::endl;
            } else if (input == "kI") {
                std::cout << kI << std::endl;
            } else if (input == "kD") {
                std::cout << kD << std::endl;
            } else if (input == "totalError") {
                std::cout << totalError << std::endl;
            } else if (input.find("kF_") == 0) {
                input.erase(0, 3);
                kF = std::stof(input);
            } else if (input.find("kA_") == 0) {
                input.erase(0, 3);
                kA = std::stof(input);
            } else if (input.find("kP_") == 0) {
                input.erase(0, 3);
                kP = std::stof(input);
            } else if (input.find("kI_") == 0) {
                input.erase(0, 3);
                kI = std::stof(input);
            } else if (input.find("kD_") == 0) {
                input.erase(0, 3);
                kD = std::stof(input);
            }
            // clear the input
            input = "";
        }
        // release the logging mutex
        logMutex.give();
    }
}


int drivePID(int driveDistance){
    //PID constants
    float kP = 0.5;
    float kI = 0;
    float kD = 0;
    //Variables for drive PID
    float error = 0;      //how far the robot is from the target
    float integral = 0;   //area under the error vs time graph
    float derivative = 0; //slope of the error vs time graph
    float prevError = 0;  //the error for the previous iteration of the PID loop

    //Motor power variables
    float motorPower = 0;     //how much power to apply to the motors, ranging from -1 (backwards at full power) to 1 (forwards at full power)
    float prevMotorPower = 0; //the motor power for the previous iteration of the PID loop
    //Reset motor encoders
    rF.set_zero_position(0);
    rB1.set_zero_position(0);
    rB2.set_zero_position(0);
    lF.set_zero_position(0);
    lB1.set_zero_position(0);
    lB2.set_zero_position(0);

    while(true){
        float currentDistance = (rF.get_position() + rB1.get_position() + rB2.get_position() + lF.get_position() + lB1.get_position() + lB2.get_position());
        pros::delay(20);
        error = driveDistance - currentDistance;
        if(error < 200 && error > -200){
            integral += error;
        }
        derivative = error - prevError;
        motorPower = (kP * error) + (kI * integral) + (kD * derivative); //calculate motor power
        if(motorPower > 1) motorPower = 1;
        if(motorPower < -1) motorPower = -1;
        
        float slewRate = 0.1f;
        if(motorPower > prevMotorPower + slewRate) motorPower = prevMotorPower + slewRate;
        if(motorPower < prevMotorPower - slewRate) motorPower = prevMotorPower - slewRate;

        lF.move_voltage(-12000 * motorPower);
        lB1.move_voltage(-12000 * motorPower);
        lB2.move_voltage(-12000 * motorPower);
        rF.move_voltage(12000 * motorPower);
        rB1.move_voltage(12000 * motorPower);
        rB2.move_voltage(12000 * motorPower);
        //update "previous" variables
        prevMotorPower = motorPower;
        prevError = error;

        //Exit the PID if the robot is within 10 degrees of the target
        if (error > -10 && error < 10) {
            break;
        }
    }
    //stop the motors when the PID is done
    lF.move_voltage(0);
    lB1.move_voltage(0);
    lB2.move_voltage(0);
    rF.move_voltage(0);
    rB1.move_voltage(0);
    rB2.move_voltage(0);
    return 0;
}

int turnPID(int turnDistance){
    //PID constants
    float kP = 0.05;
    float kI = 0;
    float kD = 0;

    //Variables for turn PID
    float error = 0;      //how far the robot is from the target, in degrees
    float integral = 0;   //area under the error vs time graph
    float derivative = 0; //slope of the error vs time graph
    float prevError = 0;  //the error for the previous iteration of the PID loop

    //Motor power variables
    float motorPower = 0;     //how much power to apply to the motors, ranging from -1 (clockwise at full power) to 1 (counterclockwise at full power)
    float prevMotorPower = 0; //the motor power for the previous iteration of the PID loop

    float startDistance = imu.get_rotation();  

    while(true) {
    //Calculate the current distance of the robot and store it as a number (float)
        float currentDistance = startDistance - imu.get_rotation();

        pros::delay(20);//don't hog CPU, this stays at the end of the loop
        error = turnDistance - currentDistance; //calculate error
        if (error < 10 && error > -10) {
            integral += error; //updated the integral term if |error| < 10
        }
        derivative = error - prevError; //calculate the derivative term
        motorPower = (kP * error) + (kI * integral) + (kD * derivative); //calculate motor power
        //keep motor power between -1 and 1
        if (motorPower > 1) motorPower = 1;
        if (motorPower < -1) motorPower = -1;

        //slew rate limiter
        float slewRate = 0.1f;
        if (motorPower > prevMotorPower + slewRate) motorPower = prevMotorPower + slewRate;
        if (motorPower < prevMotorPower - slewRate) motorPower = prevMotorPower - slewRate;

        lF.move_voltage(-12000 * motorPower);
        lB1.move_voltage(-12000 * motorPower);
        lB2.move_voltage(-12000 * motorPower);
        rF.move_voltage(12000 * motorPower);
        rB1.move_voltage(12000 * motorPower);
        rB2.move_voltage(12000 * motorPower);
        //update "previous" variables
        prevMotorPower = motorPower;
        prevError = error;

        //Exit the PID if the robot is within 10 degrees of the target
        if (error > -1 && error < 1) {
            break;
        }
    }

    //stop the motors when the PID is done
    lF.move_voltage(0);
    lB1.move_voltage(0);
    lB2.move_voltage(0);
    rF.move_voltage(0);
    rB1.move_voltage(0);
    rB2.move_voltage(0);
      
    return 0;
}