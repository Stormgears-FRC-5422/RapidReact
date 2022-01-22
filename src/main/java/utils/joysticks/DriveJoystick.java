package utils.joysticks;


public interface DriveJoystick {
    double getXSpeed();
    double getZRotation();

    double getLeftSpeed();
    double getRightSpeed();
    double getRightTrigger();
    double getLeftTrigger();
}
