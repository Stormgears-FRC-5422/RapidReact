package frc.robot.commands.climber.hold;

import frc.robot.subsystems.climber.ClimbingSubsystem;

public class HoldCurrentPosition extends HoldInterface {

  public HoldCurrentPosition(ClimbingSubsystem subsystem) {
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
