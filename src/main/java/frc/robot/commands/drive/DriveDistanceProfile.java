package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.utils.drive.StormDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveDistanceProfile extends TrapezoidProfileCommand {
    StormDrive m_drive;

    public DriveDistanceProfile(TrapezoidProfile profile,StormDrive drive) {
        super(profile,state -> execute_pid(drive,state),drive);
        m_drive = drive;
    }

    public DriveDistanceProfile(double distance, double velocity, double acceleration,StormDrive drive) {
        super(new TrapezoidProfile(new TrapezoidProfile.Constraints(velocity,acceleration),new TrapezoidProfile.State(distance,0)),
              state -> execute_pid(drive,state),
              drive);
        m_drive = drive;
    }

    public void initialize() {
        super.initialize();
        m_drive.setBrakeMode();
        m_drive.getDifferentialDrive().setSafetyEnabled(false);
        m_drive.resetPosition();
    }

    private static void execute_pid(StormDrive drive,TrapezoidProfile.State state) {
        drive.setPositionReferenceWithVelocity(state.position,state.velocity);

    }
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_drive.getDifferentialDrive().setSafetyEnabled(true);
        m_drive.getDifferentialDrive().arcadeDrive(0, 0);
        m_drive.getDifferentialDrive().setSafetyEnabled(true);
        m_drive.setCoastMode();  // should restore the mode instead of assuming
        SmartDashboard.putString("DriveDistance", "finished");
    }

}



