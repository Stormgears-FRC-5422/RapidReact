package utils.drive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface Drive {
    StormMotorType motorType();
    DifferentialDrive getDifferentialDrive();
    MotorController[] getMotors();
}
