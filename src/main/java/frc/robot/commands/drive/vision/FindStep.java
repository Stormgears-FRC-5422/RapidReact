package frc.robot.commands.drive.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.VisionDrive;

import java.util.function.Supplier;

public class FindStep extends CommandBase {
    private final Supplier<Boolean> hasTarget;
    private final VisionDrive visionDrive;

    /**
     * @param hasTarget expects vision::hasTarget
     * @param visionDrive wants a drive subsystem who implements visionDrive
     */
    public FindStep(Supplier<Boolean> hasTarget, VisionDrive visionDrive) {
        this.hasTarget = hasTarget;
        this.visionDrive = visionDrive;
    }

    @Override
    public void execute() {
        super.execute();
        visionDrive.findTarget();
    }

    @Override
    public boolean isFinished() {
        return hasTarget.get();
    }
}
