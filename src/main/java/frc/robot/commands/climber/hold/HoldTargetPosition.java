package frc.robot.commands.climber.hold;

import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.annotations.Log;

@Log.Exclude
public class HoldTargetPosition extends HoldInterface {
  @Log.Exclude ClimbingSubsystem subsystem;

  public HoldTargetPosition(ClimbingSubsystem subsystem, double position) {
    super(subsystem);
    this.subsystem = subsystem;
    updateTargetPosition(position);
  }

  public void updateTargetPosition(double position) {
    initialLeftPosition = position;
    initialRightPosition = position;
  }
}
