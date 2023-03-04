#pragma once

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include <frc/Xboxcontroller.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Solenoid.h>

#include "Constants&Defaults.h"

using ctre::phoenix::motorcontrol::can::WPI_TalonFX;

class Arm {

    private:

    bool claw_isShut = false;

    int armPosOffset = 0;

    public:

    Arm() {
        lift[0].SetInverted(true);
        lift[1].SetInverted(false);

        SmtD::PutNumber("arm setpoint", pid.GetSetpoint());

        SmtD::PutNumber("arm P", 0);
        SmtD::PutNumber("arm I", 0);
        SmtD::PutNumber("arm D", 0);

        SmtD::PutNumber("cone bottom", armLocations::cone::bottom);
        SmtD::PutNumber("cone middle", armLocations::cone::middle);
        SmtD::PutNumber("cone top"   , armLocations::cone::top   );

        SmtD::PutNumber("cube bottom", armLocations::cube::bottom);
        SmtD::PutNumber("cube middle", armLocations::cube::middle);
        SmtD::PutNumber("cube top"   , armLocations::cube::top   );

        SmtD::PutNumber("home pos"  , armLocations::home  );
        SmtD::PutNumber("pickup pos", armLocations::pickup);
    }

    WPI_TalonFX lift[2] = {
        WPI_TalonFX{CANBindings::liftL},
        WPI_TalonFX{CANBindings::liftR},
    };
    frc::Solenoid solenoids[2] = {
        frc::Solenoid(CANBindings::PnumMod, frc::PneumaticsModuleType::CTREPCM, SolenoidBindings::clawSol1),
        frc::Solenoid(CANBindings::PnumMod, frc::PneumaticsModuleType::CTREPCM, SolenoidBindings::clawSol2)
    };

    frc::MotorControllerGroup liftMs = frc::MotorControllerGroup{lift[0], lift[1]};

    frc::XboxController ctrl = frc::XboxController(USBBindings::armCtrl);

    frc2::PIDController pid{
        PIDDefaults::arm::P,
        PIDDefaults::arm::I,
        PIDDefaults::arm::D
    };
    inline int getMotorPos() {
        return lift[0].GetSelectedSensorPosition() + armPosOffset;
    }
    inline void setArmOffset() {
        armPosOffset = lift[0].GetSelectedSensorPosition();
    };

    void move() {
        //manual control
        if (ctrl.GetRightTriggerAxis() > .5) {
            pid.SetSetpoint(lift[0].GetSelectedSensorPosition());

            liftMs.Set(ctrl.GetLeftY());

            if (ctrl.GetLeftBumperPressed()) setClaw(!claw_isShut);

        }

        else liftMs.Set(pid.Calculate(lift[0].GetSelectedSensorPosition()));
    }
    int setClaw_delay = 10;
    /**
     * opens/closes the claw
     * @param isShut bool; true is shut, false is open (duh)
     * @return 1 if delay is over, 0 otherwise */
    int setClaw(bool isShut) {

        if (claw_isShut != isShut) {
            claw_isShut = isShut;
            solenoids[0].Set(isShut);
            solenoids[1].Set(isShut);
            setClaw_delay = 10;
        }
        else {
            setClaw_delay--;
            if (setClaw_delay == 0) return 1;
        }
        return 0;
    }
    int setSetPoint(bool isIncrement, int pos) {
        pid.SetSetpoint( isIncrement? getMotorPos() + pos : pos);
        return 1;
    }
    int timeSinceArival = 0;
    /**
     * function to go a predetermined location
     * @return 1 if destination has been reached, 0 otherwise
    */
    int gotoSetPoint() {
        liftMs.Set(pid.Calculate(getMotorPos()));

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