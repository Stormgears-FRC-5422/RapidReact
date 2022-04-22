package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.climber.Climber;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.kClimberMaxAccelerationSeconds;
import static frc.robot.Constants.kClimberMaxVelocity;

public class PositionClimber extends ClimbingPIDPosition {
  @Log.Exclude Climber climber;

  boolean isMovingForward = false;

  public PositionClimber(Climber climber, State goal) {
    super(
        climber,
        new Constraints(kClimberMaxVelocity, kClimberMaxVelocity * kClimberMaxAccelerationSeconds),
        goal);
    this.climber = climber;
  }

  @Override
  public void initialize() {
    super.initialize();
    isMovingForward = climber.leftPosition() >= goal.position;
    climber.disableSoftLimits();
    System.out.println("ISMOVINGFORWARD" + isMovingForward);
  }

  @Override
  public boolean isFinished() {
    if (super.isFinished()) return true;
    if (goal == ClimbingGoal.CLEARANCE_HEIGHT.getState()
        && climber.leftPosition() > goal.position
        && climber.rightPosition() > goal.position) return true;
    System.out.println(climber.isForwardLimitTripped() + "FORWARDLIMIT TRIP");
    System.out.println(climber.isReverseLimitTripped() + "FORWARDLIMIT TRIP");
    return isMovingForward ? climber.isForwardLimitTripped() : climber.isReverseLimitTripped();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    climber.enableSoftLimits();
  }
}
