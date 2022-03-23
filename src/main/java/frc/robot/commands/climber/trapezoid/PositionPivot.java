package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.climber.Pivot;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.kPivotMaxAcceleration;
import static frc.robot.Constants.kPivotMaxVelocity;

public class PositionPivot extends TrapezoidalClimbingCommand {
  @Log.Exclude Pivot pivot;

  public PositionPivot(Pivot pivot, State goalSupplier) {
    super(pivot, new Constraints(kPivotMaxVelocity, kPivotMaxAcceleration), goal);
  }
}
