package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberParentSystem;

import java.util.function.DoubleSupplier;

public class Hold extends CommandBase {

  final ClimberParentSystem subsystem;
  final DoubleSupplier leftDirection;
  final DoubleSupplier rightDirection;
  double overridePosition;
  boolean useCurrentPosition;
  double initialLeftPosition;
  double initialRightPosition;

  public Hold(ClimberParentSystem subsystem) {
    System.out.println("HOLD() 1");
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.useCurrentPosition = true;
    this.leftDirection = () -> Math.signum(initialLeftPosition - subsystem.leftPosition());
    this.rightDirection = () -> Math.signum(initialRightPosition - subsystem.rightPosition());
  }

  public Hold(ClimberParentSystem subsystem, double overridePosition) {
    System.out.println("HOLD() 2");
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.useCurrentPosition = false;
    this.overridePosition = overridePosition;
    this.leftDirection = () -> Math.signum(initialLeftPosition - subsystem.leftPosition());
    this.rightDirection = () -> Math.signum(initialRightPosition - subsystem.rightPosition());
  }

  @Override
  public void initialize() {
    if (useCurrentPosition) {
      initialLeftPosition = subsystem.leftPosition();
      initialRightPosition = subsystem.rightPosition();
    } else {
      this.initialLeftPosition = overridePosition;
      this.initialRightPosition = overridePosition;
    }
  }

  @Override
  public void execute() {
    System.out.println(
        "Left Holding: " + initialLeftPosition + " Right Holding: " + initialRightPosition);
    subsystem.leftPID(new State(initialLeftPosition, 0));
    //        subsystem.leftPID(new State(initialLeftPosition, leftDirection.getAsDouble()));
    //        subsystem.rightPID(new State(initialRightPosition, rightDirection.getAsDouble()));
    subsystem.rightPID(new State(initialRightPosition, 0));
  }
}
