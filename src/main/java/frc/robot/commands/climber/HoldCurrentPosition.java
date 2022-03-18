package frc.robot.commands.climber;

import frc.robot.subsystems.climber.ClimberParentSystem;

public class HoldCurrentPosition extends HoldInterface {

  public HoldCurrentPosition(ClimberParentSystem subsystem) {
    super(subsystem);
    holdCurrentPosition();
  }

  public void holdCurrentPosition() {
    initialLeftPosition = subsystem.leftPosition();
    initialRightPosition = subsystem.rightPosition();
  }

  @Override
  public void initialize() {
    super.initialize();
    holdCurrentPosition();
  }
}
