package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;

import java.util.function.Supplier;

public class AlignStep extends CommandBase {
    private StormDrive drive;
    private Supplier<Double> yaw;

    private PIDController alignmentPID = new PIDController(0.1, 0, 0);

    /**
     * @param getYaw expects the vision::getYaw
     * @param drive wants a drive subsystem
     */
    public AlignStep(Supplier<Double> getYaw, StormDrive drive) {
        this.drive = drive;
        yaw = getYaw;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        super.execute();
        double pidOut = alignmentPID.calculate(yaw.get()); //Supplying calculate with error
        drive.rotate(pidOut);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.rotate(0);
    }
}
