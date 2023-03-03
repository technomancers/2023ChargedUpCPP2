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
    const int clawCyl1 = 0;
    const int clawCyl2 = 2;
}

namespace PIDCoefDefaults {

    namespace drive {
        double P,I,D;
    }

    namespace arm {
        double P,I,D;
    }
}
