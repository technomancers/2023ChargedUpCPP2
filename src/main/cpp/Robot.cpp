// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  // select auton code
  autonCodeChooser.SetDefaultOption("Basic Level", auto_basicLevel);
  autonCodeChooser.AddOption("test PID", auto_testPID);
  autonCodeChooser.AddOption("Leave Community", auto_moveOut);
  autonCodeChooser.AddOption("Score and leave", auto_scoreAndLeave);
  autonCodeChooser.AddOption("Score and level", auto_scoreAndLevel);
  autonCodeChooser.AddOption("Score Level Leave", auto_scoreLevelLeave);
  autonCodeChooser.AddOption("Just Score", auto_justScore);
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

  armLocations::cone::bottom = SmtD::GetNumber("cone bottom", 0);
  armLocations::cone::middle = SmtD::GetNumber("cone middle", 0);
  armLocations::cone::top    = SmtD::GetNumber("cone top"   , 0);

  armLocations::cube::bottom = SmtD::GetNumber("cube bottom", 0);
  armLocations::cube::middle = SmtD::GetNumber("cube middle", 0);
  armLocations::cube::top    = SmtD::GetNumber("cube top"   , 0);

  armLocations::home   = SmtD::GetNumber("home pos"  , 0);
  armLocations::pickup = SmtD::GetNumber("pickup pos", 0);

  drivetrain.setOffset();
  arm.setArmOffset();

  if (autonCodeSelected == auto_testPID) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}


/**
 * auton codes are executed as a set of tasks.
 * Each code has a corrosponding int
 * The command called in each task returns whether or not it has been completed.
 * The int is incremented (++) if sucessful
 * and the next task will be executed next runthru
 */
void Robot::AutonomousPeriodic() {
  switch (autonCodeSelected) {
    case 0:// goto and level charge station

      switch (auto_basicLevel_prog) {
        case 1:
          auto_basicLevel_prog +=
          drivetrain.gotoRamp(gyro.GetXComplementaryAngle().value(), 1);
          break;

        case 2:
          auto_basicLevel_prog +=
          drivetrain.level(gyro.GetXComplementaryAngle().value());
          break;
      } break;
    case 1:// test/configure PID

      switch (auto_testPID_prog) {
        case 1:
          drivetrain.setSetPoint(false, SmtD::GetNumber("drive setpoint", 0));
          drivetrain.pid.SetPID(
            SmtD::GetNumber("drive P", 0),
            SmtD::GetNumber("drive I", 0),
            SmtD::GetNumber("drive D", 0)
          );
          SmtD::PutNumber("arm PV (position)", arm.lift[0].GetSelectedSensorPosition());
          drivetrain.gotoSetPoint();

          arm.setSetPoint(false, SmtD::GetNumber("arm setpoint", 0));
          arm.pid.SetPID(
            SmtD::GetNumber("arm P", 0),
            SmtD::GetNumber("arm I", 0),
            SmtD::GetNumber("arm D", 0)
          );
          SmtD::PutNumber("drivetrian PV (position)", drivetrain.right[0].GetSelectedSensorPosition());
          arm.gotoSetPoint();
          break;
          
      } break;
      case 3:
        switch (auto_moveOut_prog) {
          case 1:
            auto_moveOut_prog += drivetrain.setSetPoint(false, 0); // needs to be defined
            break;
          case 2:
            auto_moveOut_prog += drivetrain.gotoSetPoint();
            break;
        } break;
      case 4:
        switch (auto_scoreAndLeave_prog) {
          case 1:
            auto_scoreAndLeave_prog += arm.setSetPoint(false, armLocations::cube::middle);
            break;
          case 2:
            auto_scoreAndLeave_prog += arm.gotoSetPoint();
            break;
          case 3:
            auto_scoreAndLeave_prog += arm.setSetPoint(false, armLocations::home);
            break;
          case 4:
            auto_scoreAndLeave_prog += arm.gotoSetPoint();
            break;
          case 5:
            auto_scoreAndLeave_prog += drivetrain.setSetPoint(false, 0); // undefined
            break;
          case 6:
            auto_scoreAndLeave_prog += drivetrain.gotoSetPoint();
            break;
        } break;
      case 5:
        switch (auto_scoreAndLevel_prog) {
          case 1:
            auto_scoreAndLevel_prog += arm.setSetPoint(false, armLocations::cone::middle);
            break;
          case 2:
            auto_scoreAndLevel_prog += arm.gotoSetPoint();
            break;
          case 3:
            auto_scoreAndLevel_prog += arm.setSetPoint(false, armLocations::home);
            break;
          case 4:
            auto_scoreAndLevel_prog += arm.gotoSetPoint();
            break;
          case 5:
            auto_scoreAndLevel_prog += drivetrain.gotoRamp(gyro.GetXComplementaryAngle().value(), 1);
            break;
          case 6:
            auto_scoreAndLevel_prog += drivetrain.level(gyro.GetXComplementaryAngle().value());
            break;
        } break;
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

  	if (arm.ctrl.GetLeftTriggerAxis() < .5) {
		  if 		(arm.ctrl.GetAButtonPressed()) arm.setSetPoint(false, armLocations::cone::bottom);
		  else if (arm.ctrl.GetXButtonPressed()) arm.setSetPoint(false, armLocations::cone::middle);
		  else if (arm.ctrl.GetYButtonPressed()) arm.setSetPoint(false, armLocations::cone::top   );

		  else if (arm.ctrl.GetBButtonPressed()) arm.setSetPoint(false, armLocations::home);
    }
	  else {
		  if 		(arm.ctrl.GetAButtonPressed()) arm.setSetPoint(false, armLocations::cube::bottom);
		  else if (arm.ctrl.GetXButtonPressed()) arm.setSetPoint(false, armLocations::cube::middle);
		  else if (arm.ctrl.GetYButtonPressed()) arm.setSetPoint(false, armLocations::cube::top   );

		  else if (arm.ctrl.GetBButtonPressed()) arm.setSetPoint(false, armLocations::pickup);
		}

	if (arm.ctrl.GetBackButton() && arm.ctrl.GetStartButton()) arm.setArmOffset();

	arm.move();
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
