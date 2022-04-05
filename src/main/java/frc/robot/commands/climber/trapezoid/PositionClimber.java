package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.climber.Climber;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.kClimberMaxAccelerationSeconds;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends TrapezoidalClimbingCommand {
  @Log.Exclude Climber climber;

  public PositionClimber(Climber climber, State goal) {
    super(climber, new Constraints(kClimberMaxVelocity, kClimberMaxVelocity * kClimberMaxAccelerationSeconds), goal);
  }
}
