package frc.robot.commands.climber;

import frc.robot.subsystems.climber.ClimberParentSystem;

public class HoldTargetPosition extends HoldInterface {
  public HoldTargetPosition(ClimberParentSystem subsystem, double position) {
    super(subsystem);
    updateTargetPosition(position);
  }

  public void updateTargetPosition(double position) {
    initialLeftPosition = position;
    initialRightPosition = position;
  }
}
