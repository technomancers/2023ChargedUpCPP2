//prevent from being included twice
#ifndef NoInclude_Constants_Defaults
#define NoInclude_Constants_Defaults

#define SmtD frc::SmartDashboard



namespace USBBindings {
    const int driveCtrl = 0;
    const int armCtrl   = 1;
}

namespace CANBindings {
    const int PnumMod = 0;

    const int leftDriveF  = 5;
    const int leftDriveB  = 6;
    const int rightDriveF = 3;
    const int rightDriveB = 4;

    const int liftL = 8;
    const int liftR = 9;
}

namespace SolenoidBindings {
    const int clawSol1 = 0;
    const int clawSol2 = 2;
}

namespace PIDDefaults {

    namespace drive {
        double P = 0;
        double I = 0;
        double D = 0;
    }

    namespace arm {
        double P = 0;
        double I = 0;
        double D = 0;
    }
}

namespace armLocations {

    namespace cone {
        int bottom = 0;
        int middle = 0;
        int top    = 0;
    }

    namespace cube {
        int bottom = 0;
        int middle = 0;
        int top    = 0;
    }

    int home = 0;

    int pickup = 0;
}
#endif
