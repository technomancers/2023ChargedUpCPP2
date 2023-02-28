#include "ctre/Phoenix.h"

#include <frc/Xboxcontroller.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

class Arm {

    private:

    public:

    Arm() {
        lift[0].SetInverted(true);
        lift[1].SetInverted(false);
    }

    WPI_TalonFX lift[2] = {
        WPI_TalonFX{MCBindings::liftL},
        WPI_TalonFX{MCBindings::liftR},
    };

    frc::MotorControllerGroup liftMs = frc::MotorControllerGroup{lift[0], lift[1]};

    frc::XboxController ctrl = frc::XboxController(USBBindings::armCtrl);

    void move(int pos) {

        //manual control
        if (ctrl.GetRightTriggerAxis() > .5) liftMs.Set(ctrl.GetRightY());

        else {}
        

    };
};