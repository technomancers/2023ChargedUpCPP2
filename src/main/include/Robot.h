// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <bits/stdc++.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/ADIS16470_IMU.h>

#include "DriveTrain.h"
#include "Arm.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  double speeds[3] = {.50, .75, 1.00};
  double *speed = &speeds[2];

  DriveTrain drivetrain = DriveTrain();
  
  Arm arm = Arm();

  frc::ADIS16470_IMU gyro = frc::ADIS16470_IMU{};

 private:
  frc::SendableChooser<int> autonCodeChooser;
  const int auto_basicLevel      = 0;
  const int auto_testPID         = 1;
  const int auto_moveOut         = 2;
  const int auto_scoreAndLeave   = 3;
  const int auto_scoreAndLevel   = 4;
  const int auto_scoreLevelLeave = 5;
  const int auto_justScore       = 6;
  int autonCodeSelected;

  frc::SendableChooser<char> driveModeChooser;
  char driveModeSelected;


  int auto_basicLevel_prog = 1;
  int auto_testPID_prog = 1;
  int auto_moveOut_prog = 1;
  int auto_scoreAndLeave_prog = 1;
  int auto_scoreAndLevel_prog = 1;
  int auto_scoreLevelLeave_prog = 1;
  int auto_justScore_prog = 1;
};
