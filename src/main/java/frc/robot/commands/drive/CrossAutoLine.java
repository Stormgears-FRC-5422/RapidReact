package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SafeDrive;

public class CrossAutoLine extends CommandBase {
    private final SafeDrive drive;
    private int count;

    public CrossAutoLine(SafeDrive drive){
        addRequirements(drive);

        this.drive = drive;
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        drive.driveArcade(-0.20, 0);
        count++;
    }

    @Override
    public boolean isFinished() {
        return count >= 500;
    }
}