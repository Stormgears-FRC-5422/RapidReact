package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.utils.drive.StormDrive;

public class DriveDistanceProfile extends TrapezoidProfileCommand {
  StormDrive m_drive;

  public DriveDistanceProfile(TrapezoidProfile profile, StormDrive drive) {
    super(profile, state -> execute_pid(drive, state), drive);
    addRequirements(drive);
    m_drive = drive;
  }

  public DriveDistanceProfile(
      double distance, double velocity, double acceleration, StormDrive drive) {
    super(
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(velocity, acceleration),
            new TrapezoidProfile.State(distance, 0)),
        state -> execute_pid(drive, state),
        drive);
    addRequirements(drive);
    m_drive = drive;
  }

  private static void execute_pid(StormDrive drive, TrapezoidProfile.State state) {
    drive.setPositionReferenceWithVelocity(state.position, state.velocity);
  }

  public void initialize() {
    super.initialize();
    //    SmartDashboard.putString("DriveDistance", "Running");
    m_drive.getDifferentialDrive().setSafetyEnabled(false);
    m_drive.resetPosition();
    //    SmartDashboard.putString("DriveDistance", "Running");
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drive.getDifferentialDrive().setSafetyEnabled(true);
    m_drive.getDifferentialDrive().arcadeDrive(0, 0);
    System.out.println(
        "FINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHEDFINISHED");
    //    SmartDashboard.putString("DriveDistance", "finished");
  }
}
