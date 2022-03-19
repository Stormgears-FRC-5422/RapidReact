package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.Supplier;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public abstract class TrapezoidalClimbingCommand extends CommandBase implements Loggable {

  @Log.Exclude protected final ClimbingSubsystem subsystem;
  protected final Constraints constraints;

  protected TrapezoidProfileCommand leftTrapezoidProfileCommand;
  protected TrapezoidProfileCommand rightTrapezoidProfileCommand;

  protected Supplier<State> goal;
  private State currentGoal;

  protected TrapezoidalClimbingCommand(
      ClimbingSubsystem subsystem, Constraints constraints, Supplier<State> goal) {
    this.subsystem = subsystem;
    this.constraints = constraints;
    this.goal = goal;
    currentGoal = goal.get();

    this.leftTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, currentGoal, new State(subsystem.leftPosition(), 0)),
            this::leftPID);
    this.rightTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, currentGoal, new State(subsystem.rightPosition(), 0)),
            this::rightPID);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    updatePosition(goal.get());
    leftTrapezoidProfileCommand.initialize();
    rightTrapezoidProfileCommand.initialize();
    System.out.println("TRYING TO MOVE " + subsystem.getName() + " TO " + goal.get().position);
  }

  @Override
  public void execute() {
    leftTrapezoidProfileCommand.execute();
    rightTrapezoidProfileCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    leftTrapezoidProfileCommand.end(interrupted);
    rightTrapezoidProfileCommand.end(interrupted);
    if (!interrupted) subsystem.holdTarget(goal.get().position);
  }

  @Override
  public boolean isFinished() {
    return leftTrapezoidProfileCommand.isFinished() && rightTrapezoidProfileCommand.isFinished();
  }

  protected void leftPID(State state) {
    subsystem.leftPID(state);
  }

  protected void rightPID(State state) {
    subsystem.rightPID(state);
  }

  public void updatePosition(State newPosition) {
    currentGoal = newPosition;
    this.leftTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, currentGoal, new State(subsystem.leftPosition(), 0)),
            this::leftPID);
    this.rightTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, currentGoal, new State(subsystem.rightPosition(), 0)),
            this::rightPID);
  }

  @Log(name = "Current Goal")
  double goalPosition() {
    return currentGoal.position;
  }
}
