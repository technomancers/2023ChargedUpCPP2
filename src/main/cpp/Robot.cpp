// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

#include <frc/MotorSafety.h>

void Robot::RobotInit() {
  // select auton code
  autonCodeChooser.SetDefaultOption("Basic Level", auto_basicLevel);
  autonCodeChooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes: ", &autonCodeChooser);

  driveModeChooser.SetDefaultOption("Arcade Drive", 'a');
  driveModeChooser.AddOption("Tank Drive", 't');
  frc::SmartDashboard::PutData("Drive Modes: ", &driveModeChooser);

  frc::SmartDashboard::PutString("Front : ", drivetrain.front[drivetrain.dir]);
  frc::SmartDashboard::PutNumber("Speed Mult % : ", *speed * 100);
  frc::SmartDashboard::PutNumber("Right Compensation : ", drivetrain.rightCompensation);

  frc::CameraServer::StartAutomaticCapture();

  gyro.Calibrate();
  }

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  // get auton code
  autonCodeSelected = autonCodeChooser.GetSelected();

  fmt::print("Auto selected: {}\n", autonCodeSelected);

  if (autonCodeSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}


/**
 * auton codes are executed as a set of tasks.
 * Each code has a corrosponding bool[] containing its progress.
 * The command called in each task returns whether or not it has bee completed.
 * The array is then updated so the next run proceeds to the next task
*/
void Robot::AutonomousPeriodic() {
  if (autonCodeSelected == auto_basicLevel) {

    if (!auto_basicLevel_prog[0]) auto_basicLevel_prog[0] = 
    drivetrain.gotoRamp(gyro.GetXComplementaryAngle().value());

    else if (!auto_basicLevel_prog[1])  auto_basicLevel_prog[1] = 
    drivetrain.level(gyro.GetXComplementaryAngle().value());
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  // get drive mode
  driveModeSelected = driveModeChooser.GetSelected();

  
  drivetrain.rightCompensation = 
  frc::SmartDashboard::GetNumber("Right Compensation : " , drivetrain.rightCompensation);

}

void Robot::TeleopPeriodic() {
  // get speed multiplier
  if      (drivetrain.ctrl.GetLeftStickButtonPressed()  and speed != speeds    ) speed--;
  else if (drivetrain.ctrl.GetRightStickButtonPressed() and speed != speeds + 2) speed++;

  frc::SmartDashboard::PutNumber("Speed Mult % : ", *speed * 100);

  //reverse drive if y pressed
  if (drivetrain.ctrl.GetYButtonPressed()) {
    drivetrain.dir *= -1;
    frc::SmartDashboard::PutString("Front : ", drivetrain.front[drivetrain.dir]);
  }
  // drive
  drivetrain.drive(*speed, driveModeSelected);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
