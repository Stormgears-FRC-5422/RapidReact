package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberParentSystem;

public abstract class HoldInterface extends CommandBase {
  final ClimberParentSystem subsystem;
  double initialLeftPosition;
  double initialRightPosition;

  HoldInterface(ClimberParentSystem subsystem) {
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
