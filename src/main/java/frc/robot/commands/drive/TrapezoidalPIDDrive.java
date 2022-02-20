package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SafeDrive;
import frc.utils.drive.StormDrive;

public class TrapezoidalPIDDrive extends TrapezoidProfileCommand {

    private StormDrive drive;
    private TrapezoidProfile.State goal;

    public TrapezoidalPIDDrive(StormDrive _drive, TrapezoidProfile.State goal) {
        super(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(Constants.kMAXVEL, Constants.kMAXACC)
                    , goal),
                (TrapezoidProfile.State output) -> {
                    double pidOffset = _drive.calculateDriveVel(output.position);
                    //TODO: characterize the robot velocity -> %output
                    double actualOut = output.velocity + pidOffset;
                    _drive.getDifferentialDrive().arcadeDrive(actualOut, 0, false);
                },
            _drive
        );
        drive = _drive;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        super.initialize();
        drive.ResetEncoders();
    }

    @Override
    public boolean isFinished() {
        return drive.getDistance() >= goal.position;
    }
}
