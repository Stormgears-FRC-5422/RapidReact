package frc.robot.commands.climber.hold;

import frc.robot.subsystems.climber.ClimbingSubsystem;

public class HoldTargetPosition extends HoldInterface {
  public HoldTargetPosition(ClimbingSubsystem subsystem, double position) {
    super(subsystem);
    updateTargetPosition(position);
  }

  public void updateTargetPosition(double position) {
    initialLeftPosition = position;
    initialRightPosition = position;
  }
}
