package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.climber.Climber;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.Supplier;

import static frc.robot.Constants.kClimberMaxAcceleration;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends TrapezoidalClimbingCommand {
  @Log.Exclude Climber climber;

  public PositionClimber(Climber climber, Supplier<TrapezoidProfile.State> goalSupplier) {
    super(climber, new Constraints(kClimberMaxVelocity, kClimberMaxAcceleration), goalSupplier);
  }
}
