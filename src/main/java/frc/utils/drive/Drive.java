package frc.utils.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drive extends Subsystem {
    StormMotorType motorType();
    DifferentialDrive getDifferentialDrive();
    MotorController[] getMotors();

  void rotate(double zRotation);
}
