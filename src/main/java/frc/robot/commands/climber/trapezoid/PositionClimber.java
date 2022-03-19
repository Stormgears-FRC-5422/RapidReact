package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.climber.Climber;

import java.util.function.Supplier;

import static frc.robot.Constants.kClimberMaxAcceleration;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends TrapezoidalClimbingCommand {

  public PositionClimber(Climber climber, Supplier<TrapezoidProfile.State> goalSupplier) {
    super(climber, new Constraints(kClimberMaxVelocity, kClimberMaxAcceleration), goalSupplier);
  }
}
