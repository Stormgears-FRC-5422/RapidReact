package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.climber.Pivot;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.kPivotMaxAccelerationSeconds;
import static frc.robot.Constants.kPivotMaxVelocity;

public class PositionPivot extends ClimbingPIDPosition {
  @Log.Exclude Pivot pivot;

  public PositionPivot(Pivot pivot, State goal) {
    super(
        pivot,
        new Constraints(kPivotMaxVelocity, kPivotMaxVelocity * kPivotMaxAccelerationSeconds),
        goal);
    if (goal == PivotGoal.SECOND.getState())
      constraints = new Constraints(constraints.maxVelocity * 2d / 3d, constraints.maxAcceleration);
  }
}
