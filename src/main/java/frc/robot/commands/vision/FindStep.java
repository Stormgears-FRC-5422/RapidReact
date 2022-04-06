package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;

import java.util.function.Supplier;

public class FindStep extends CommandBase {
    private Supplier<Boolean> hasTarget;
    private StormDrive drive;

    /**
     * @param hasTarget expects vision::hasTarget
     * @param drive wants a drive subsystem
     */
    public FindStep(Supplier<Boolean> hasTarget, StormDrive drive) {
        this.hasTarget = hasTarget;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        super.execute();
        drive.rotate(0.7);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.rotate(0);
    }

    @Override
    public boolean isFinished() {
        return hasTarget.get();
    }
}
