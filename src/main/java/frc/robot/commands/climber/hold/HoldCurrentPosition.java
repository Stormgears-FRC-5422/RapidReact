package frc.robot.commands.climber.hold;

import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class HoldCurrentPosition extends HoldInterface {

  @Log.Exclude ClimbingSubsystem subsystem;

  public HoldCurrentPosition(ClimbingSubsystem subsystem) {
    super(subsystem);
    this.subsystem = subsystem;
    //holdCurrentPosition();
  }

  public void holdCurrentPosition() {
    initialLeftPosition = subsystem.leftPosition();
    initialRightPosition = subsystem.rightPosition();
  }

  @Override
  public void initialize() {
    holdCurrentPosition();
  }
}
