package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public abstract class TrapezoidalClimbingCommand extends CommandBase implements Loggable {

  @Log.Exclude protected final ClimbingSubsystem subsystem;
  protected final Constraints constraints;

  protected TrapezoidProfileCommand leftTrapezoidProfileCommand;
  protected TrapezoidProfileCommand rightTrapezoidProfileCommand;

  private final State goal;

  protected TrapezoidalClimbingCommand(
      ClimbingSubsystem subsystem, Constraints constraints, State goal) {
    this.subsystem = subsystem;
    this.constraints = constraints;
    this.goal = goal;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    this.leftTrapezoidProfileCommand =
            new TrapezoidProfileCommand(
                    new TrapezoidProfile(constraints, goal, new State(subsystem.leftPosition(), 0)),
                    this::leftPID);
    this.rightTrapezoidProfileCommand =
            new TrapezoidProfileCommand(
                    new TrapezoidProfile(constraints, goal, new State(subsystem.rightPosition(), 0)),
                    this::rightPID);
    leftTrapezoidProfileCommand.initialize();
    rightTrapezoidProfileCommand.initialize();
    System.out.println("TRYING TO MOVE " + subsystem.getName() + " TO " + goal);
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
    subsystem.stop();
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

  @Log(name = "Current Goal")
  double goalPosition() {
    return goal.position;
  }
}
