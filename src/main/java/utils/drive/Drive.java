package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import utils.drive.StormMotorType;

public interface Drive {
    public StormMotorType motorType();
    public DifferentialDrive getDifferentialDrive();
}
