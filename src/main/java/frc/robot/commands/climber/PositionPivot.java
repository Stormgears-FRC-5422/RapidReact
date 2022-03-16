package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.climber.Pivot;

import static frc.robot.Constants.kPivotMaxAcceleration;
import static frc.robot.Constants.kPivotMaxVelocity;

public class PositionPivot extends MoveCommand {
    public PositionPivot(Pivot pivot) {
    super(pivot, new Constraints(kPivotMaxVelocity, kPivotMaxAcceleration));
    }
}
