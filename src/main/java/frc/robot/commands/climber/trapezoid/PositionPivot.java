package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.climber.Pivot;

import java.util.function.Supplier;

import static frc.robot.Constants.kPivotMaxAcceleration;
import static frc.robot.Constants.kPivotMaxVelocity;

public class PositionPivot extends TrapezoidalClimbingCommand {
  public PositionPivot(Pivot pivot, Supplier<State> goalSupplier) {
    super(pivot, new Constraints(kPivotMaxVelocity, kPivotMaxAcceleration), goalSupplier);
  }
}
