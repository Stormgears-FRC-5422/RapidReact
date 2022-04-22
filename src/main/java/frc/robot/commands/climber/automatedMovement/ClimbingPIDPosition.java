package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public abstract class ClimbingPIDPosition extends CommandBase implements Loggable {

  @Log.Exclude protected final ClimbingSubsystem subsystem;
  protected final State goal;
  private final State leftGoal;
  private final State rightGoal;
  protected Constraints constraints;

  protected ClimbingPIDPosition(ClimbingSubsystem subsystem, Constraints constraints, State goal) {
    this.subsystem = subsystem;
    this.constraints = constraints;
    this.goal = goal;

    leftGoal = new State(goal.position, 0);
    rightGoal = new State(goal.position, 0);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("TRYING TO MOVE " + subsystem.getName() + " TO " + goal);
    subsystem.resetPID();
  }

  @Override
  public void execute() {
    // TODO Constant
    leftGoal.velocity =
        (Math.abs(subsystem.leftPosition() - goal.position) > 0.03 ? 1 : 0)
            * constraints.maxVelocity
            * (goal.position > subsystem.leftPosition() ? 1 : -1);
    rightGoal.velocity =
        (Math.abs(subsystem.rightPosition() - goal.position) > 0.03 ? 1 : 0)
            * constraints.maxVelocity
            * (goal.position > subsystem.rightPosition() ? 1 : -1);

    subsystem.leftPID(leftGoal);
    subsystem.rightPID(rightGoal);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO Constant
    return Math.abs(goal.position - subsystem.leftPosition()) <= 0.008
        && Math.abs(goal.position - subsystem.rightPosition()) <= 0.008;
  }

  @Log(name = "Current Goal")
  double goalPosition() {
    return goal.position;
  }
}
