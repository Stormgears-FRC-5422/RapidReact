package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;

public interface VisionDrive {
    default PIDController getController() {
        return new PIDController(kVisionDriveP, kVisionDriveI, kVisionDriveD);
    }

    void setError(double yaw);
    void findTarget();
}