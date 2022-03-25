package frc.robot.commands.climber.hold;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public abstract class HoldInterface extends CommandBase {

  @Log.Exclude final ClimbingSubsystem subsystem;

  @Log(name = "Holding Left Position")
  double initialLeftPosition;

  @Log(name = "Holding Right Position")
  double initialRightPosition;



  HoldInterface(ClimbingSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.leftPID(new TrapezoidProfile.State(initialLeftPosition, 0));
    subsystem.rightPID(new TrapezoidProfile.State(initialRightPosition, 0));
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }
}
