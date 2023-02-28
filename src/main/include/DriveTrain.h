#pragma once

#include <string>
#include <map>
#include <bits/stdc++.h>

#include <frc/Xboxcontroller.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

#include "ctre/Phoenix.h"

#include "Constants.h"

class DriveTrain {

    public:

    DriveTrain() {
        front[1 ] = "hopper";
        front[-1] = "arm";

        // invert drive on right since motors face opposite dirrections
        rightMs.SetInverted(true);
        leftMs.SetInverted(false);
    }

    // left motors when hopper is front
    WPI_TalonFX left [2] = {
        WPI_TalonFX{MCBindings::leftDriveF},
        WPI_TalonFX{MCBindings::leftDriveB}};
    frc::MotorControllerGroup leftMs = frc::MotorControllerGroup{left[0], left[1]};
    // right motors when hopper is front

    WPI_TalonFX right[2] = {
        WPI_TalonFX{MCBindings::rightDriveF},
        WPI_TalonFX{MCBindings::rightDriveB}};
    frc::MotorControllerGroup rightMs = frc::MotorControllerGroup{right[0], right[1]};

    // frc::DifferentialDrives for forward and reverse
    frc::DifferentialDrive diffDrives[2] = {
        frc::DifferentialDrive{leftMs, rightMs},
        frc::DifferentialDrive{rightMs, leftMs}};


    frc::XboxController ctrl = frc::XboxController(USBBindings::driveCtrl);

    int dir = 1;

    /**
     * map <1 : "hopper">, <-1 : "arm">
     * tells which end is the front
     * 
     * assigned in Robot::RobotInit()
    */
    std::map<int,std::string> front;


    double rightCompensation = .93;

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
    void level(double angle) {
        
        if (angle < 5 and angle > -5) SmtD::PutString("direction", "flat");
        
        else if (angle > 5) {
            SmtD::PutString("direction", "hopper up");
        }
        else {
            SmtD::PutString("direction", "arm up");
        }

        diffDrives[0].Feed();
        diffDrives[1].Feed();

        frc::SmartDashboard::PutNumber("Angle", angle);


    };
};