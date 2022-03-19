package frc.robot.commands.climber.hold;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public abstract class HoldInterface extends CommandBase implements Loggable {

  @Log.Exclude final ClimbingSubsystem subsystem;

  @Log(name = "Holding Left Position")
  double initialLeftPosition;

  @Log(name = "Holding Right Position")
  double initialRightPosition;

  @Override
  public String configureLogName() {
    return subsystem.getName();
  }

  HoldInterface(ClimbingSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.leftPID(new TrapezoidProfile.State(initialLeftPosition, 0));
    subsystem.rightPID(new TrapezoidProfile.State(initialRightPosition, 0));
    // System.out.println(
    //   subsystem.getName() + " Left Holding: " + initialLeftPosition + " Right Holding: " +
    // initialRightPosition);
    //        subsystem.leftPID(new State(initialLeftPosition, leftDirection.getAsDouble()));
    //        subsystem.rightPID(new State(initialRightPosition, rightDirection.getAsDouble()));
  }
}
