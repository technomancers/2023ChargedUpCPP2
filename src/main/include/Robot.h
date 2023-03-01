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
  frc::SendableChooser<std::string> autonCodeChooser;
  const std::string auto_basicLevel = "Basic Level";
  const std::string kAutoNameCustom = "My Auto";
  std::string autonCodeSelected;

  frc::SendableChooser<char> driveModeChooser;
  char driveModeSelected;


  bool auto_basicLevel_prog[2] = {false};
};
