package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.climber.Climber;

import static frc.robot.Constants.kClimberMaxAcceleration;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends TrapezoidalClimbingCommand {

  public PositionClimber(Climber climber, TrapezoidProfile.State goal) {
    super(climber, new Constraints(kClimberMaxVelocity, kClimberMaxAcceleration), goal);
    }

}
