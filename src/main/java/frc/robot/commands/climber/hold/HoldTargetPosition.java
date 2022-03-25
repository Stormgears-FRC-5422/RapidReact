package frc.robot.commands.climber.hold;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.annotations.Log;

@Log.Exclude
public class HoldTargetPosition extends HoldInterface {
  @Log.Exclude ClimbingSubsystem subsystem;
  private final double position;

  public HoldTargetPosition(ClimbingSubsystem subsystem, double position) {
    super(subsystem);
    this.subsystem = subsystem;
    this.position = position;
    updateTargetPosition(position);
  }

  public void updateTargetPosition(double position) {
    this.initialLeftPosition = position;
    this.initialRightPosition = position;
  }

  @Override
  public void initialize() {
    updateTargetPosition(position);
  }

  @Override
  public void execute() {
    subsystem.leftPID(new TrapezoidProfile.State(position, 0));
    subsystem.rightPID(new TrapezoidProfile.State(position, 0));
  }
}
