package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public abstract class TrapezoidalClimbingCommand extends CommandBase implements Loggable {

  @Log.Exclude protected final ClimbingSubsystem subsystem;
  protected final Constraints constraints;

  //  protected TrapezoidProfileCommand leftTrapezoidProfileCommand;
  //  protected TrapezoidProfileCommand rightTrapezoidProfileCommand;

  protected final State goal;

  private final State leftGoal;
  private final State rightGoal;

  protected TrapezoidalClimbingCommand(
      ClimbingSubsystem subsystem, Constraints constraints, State goal) {
    this.subsystem = subsystem;
    this.constraints = constraints;
    this.goal = goal;

    leftGoal = new State(goal.position, 0);
    rightGoal = new State(goal.position, 0);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //    this.leftTrapezoidProfileCommand =
    //            new TrapezoidProfileCommand(
    //                    new TrapezoidProfile(constraints, goal, new
    // State(subsystem.leftPosition(), 0)),
    //                    this::leftPID);
    //    this.rightTrapezoidProfileCommand =
    //            new TrapezoidProfileCommand(
    //                    new TrapezoidProfile(constraints, goal, new
    // State(subsystem.rightPosition(), 0)),
    //                    this::rightPID);
    //    leftTrapezoidProfileCommand.initialize();
    //    rightTrapezoidProfileCommand.initialize();
    System.out.println("TRYING TO MOVE " + subsystem.getName() + " TO " + goal);
    subsystem.resetPID();
  }

  @Override
  public void execute() {
    //    leftTrapezoidProfileCommand.execute();
    //    rightTrapezoidProfileCommand.execute();
    leftGoal.velocity =
        (Math.abs(subsystem.leftPosition() - goal.position) > 0.05 ? 1 : 0)
            * constraints.maxVelocity
            * (goal.position > subsystem.leftPosition() ? 1 : -1);
    rightGoal.velocity =
        (Math.abs(subsystem.rightPosition() - goal.position) > 0.05 ? 1 : 0)
            * constraints.maxVelocity
            * (goal.position > subsystem.rightPosition() ? 1 : -1);

    subsystem.leftPID(leftGoal);
    subsystem.rightPID(rightGoal);
  }

  @Override
  public void end(boolean interrupted) {
    //    leftTrapezoidProfileCommand.end(interrupted);
    //    rightTrapezoidProfileCommand.end(interrupted);
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    //    return leftTrapezoidProfileCommand.isFinished() &&
    // rightTrapezoidProfileCommand.isFinished();
    return Math.abs(goal.position - subsystem.leftPosition()) <= 0.003
        && Math.abs(goal.position - subsystem.rightPosition()) <= 0.003;
  }

  protected void leftPID(double unused, State state) {
    subsystem.leftPID(state);
  }

  protected void rightPID(double unused, State state) {
    subsystem.rightPID(state);
  }

  @Log(name = "Current Goal")
  double goalPosition() {
    return goal.position;
  }
}
