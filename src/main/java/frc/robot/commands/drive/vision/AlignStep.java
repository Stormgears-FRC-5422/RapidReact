package frc.robot.commands.drive.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.VisionDrive;
import frc.utils.drive.StormDrive;

import java.util.function.Supplier;

public class AlignStep extends CommandBase {
    private final VisionDrive drive;
    private final Supplier<Double> yaw;

    /**
     * @param getYaw expects the vision::getYaw
     * @param visionDrive wants a drive subsystem who implements visionDrive
     */
    public AlignStep(Supplier<Double> getYaw, VisionDrive visionDrive) {
        this.drive = visionDrive;
        yaw = getYaw;
    }

    @Override
    public void execute() {
        super.execute();
        drive.setError(yaw.get());
    }
}
