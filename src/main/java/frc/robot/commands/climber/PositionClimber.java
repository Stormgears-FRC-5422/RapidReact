package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.climber.Climber;

import static frc.robot.Constants.kClimberMaxAcceleration;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends MoveCommand {
    public PositionClimber(Climber climber) {
    super(climber, new Constraints(kClimberMaxVelocity, kClimberMaxAcceleration));
    }
}
