#pragma once

#include <map>
#include <bits/stdc++.h>

#include <frc/Xboxcontroller.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

#include <frc/controller/PIDController.h>

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include "Constants&Defaults.h"

using ctre::phoenix::motorcontrol::can::WPI_TalonFX;

class DriveTrain {

    public:

    DriveTrain() {
        front[ 1] = "hopper";
        front[-1] = "arm";

        // invert drive on right since motors face opposite dirrections
        rightMs.SetInverted(true);
        leftMs.SetInverted(false);

        SmtD::PutNumber("drive setpoint", pid.GetSetpoint());

        SmtD::PutNumber("drive P", 0);
        SmtD::PutNumber("drive I", 0);
        SmtD::PutNumber("drive D", 0);
    }

    // left motors when hopper is front
    WPI_TalonFX left [2] = {
        WPI_TalonFX{CANBindings::leftDriveF},
        WPI_TalonFX{CANBindings::leftDriveB}};
    frc::MotorControllerGroup leftMs = frc::MotorControllerGroup{left[0], left[1]};
    // right motors when hopper is front

    WPI_TalonFX right[2] = {
        WPI_TalonFX{CANBindings::rightDriveF},
        WPI_TalonFX{CANBindings::rightDriveB}};
    frc::MotorControllerGroup rightMs = frc::MotorControllerGroup{right[0], right[1]};

    // frc::DifferentialDrives for forward and reverse
    frc::DifferentialDrive diffDrives[2] = {
        frc::DifferentialDrive{leftMs, rightMs},
        frc::DifferentialDrive{rightMs, leftMs}};


    frc::XboxController ctrl = frc::XboxController(USBBindings::driveCtrl);

    int dir = 1;

    frc2::PIDController pid {
        PIDDefaults::drive::P,
        PIDDefaults::drive::I,
        PIDDefaults::drive::D};



    /**
     * map <1 : "hopper">, <-1 : "arm">
     * tells which end is the front
     * 
     * assigned in Robot::RobotInit()
    */
    std::map<int,std::string> front;


    double rightCompensation = .93;

    /**
     * function to get the output of the PID loop
     * @return the output of the PID loop based on the right front motor position
    */
    inline double getPIDOutput() {
        return pid.Calculate(right[0].GetSelectedSensorVelocity());
    }

    /**
     * makes the robot drive
     * @param vel the speed multiplier
     * @param mode drive mode, 'a' = arcade, 't' = tank
    */
    void drive(double vel, char mode) {
        if (ctrl.GetRightTriggerAxis() > .5) vel = 1;
        
        if (mode == 'a') {
            diffDrives[(dir+1)/2].ArcadeDrive(ctrl.GetLeftY() *vel*dir, ctrl.GetRightX()*vel*dir);
         
            if (dir == 1) rightMs.Set(rightMs.Get() * rightCompensation);
            else leftMs.Set(leftMs.Get() * rightCompensation);
        }

        else if (mode == 't')
            diffDrives[(dir+1)/2].TankDrive(ctrl.GetLeftY() *vel*dir, ctrl.GetRightY()*vel*dir*rightCompensation);


        // keep motorsafety from being a problem
        diffDrives[0].Feed();
        diffDrives[1].Feed();
    }

    /**
     * functions below are used in Robot::AutonomousPeriodic().
     * they return 1 if their task is completed so that the nex
     * run will use the next task. otherwise, they return 0 and
     * are reexectued next cycle
    */


    int leveltime = 0;

    /**
     * function to automatically level the charge station
     * @param angle the pich of the robot in degrees
     * @return 1 if level, 0 otherwise
    */
    int level(double angle) {
        
        if (angle < 8 and angle > -8) {
            SmtD::PutString("direction", "flat");
            diffDrives[0].TankDrive(0,0);
            leveltime++;
        }
        else if (angle >= 8) {
            SmtD::PutString("direction", "arm up");
            diffDrives[0].TankDrive(.65,.65 * rightCompensation);
        }
        else {
            SmtD::PutString("direction", "hopper up");
            diffDrives[0].TankDrive(-.65,-.65 * rightCompensation);
        }
        diffDrives[1].Feed();

        frc::SmartDashboard::PutNumber("Angle", angle);

        if (leveltime >= 100) return 1;
        else return 0;
    };
    /**
     * function to drive to the chrage station
     * @param angle the pitch of the robot in degrees
     * @param dir 1 for forward, -1 for backward
     * @return 1 if the charge station has been reached, 0 otherwise
    */
    int gotoRamp(double angle, int _dir) {
        if (angle > 8 or angle < -8) {
            diffDrives[0].Feed();
            diffDrives[1].Feed();
            return 1;
        }
        else {
            diffDrives[0].TankDrive(-.65 * _dir,-.65 * _dir * rightCompensation);
            diffDrives[1].Feed();
            return 0;
        }
    }
    /**
     * function to set setpoint of pid loop.
     * exists only to make auton codes work correctly
     * @param isIncrement whenther or not to add the current position to pos
     * @param pos where/how far to go
     * @return 1
    */
    int setSetPoint(bool isIncrement, int pos) {
        pid.SetSetpoint( isIncrement? right[0].GetSelectedSensorPosition() + pos : pos);
        return 1;
    }

    int timeSinceArival = 0;
    /**
     * function to drive a predetermined location
     * @return 1 if destination has been reached, 0 otherwise
    */
    int gotoSetPoint() {
        diffDrives[0].TankDrive(getPIDOutput(),getPIDOutput() * rightCompensation);
        diffDrives[1].Feed();

        if (pid.AtSetpoint()) {
            timeSinceArival++;
            if (timeSinceArival > 8) {
                timeSinceArival = 0;
                return 1;
            }
        }
        else timeSinceArival = 0;
        return 0;
   };
};