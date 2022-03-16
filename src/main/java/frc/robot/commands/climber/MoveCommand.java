package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimberParentSystem;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public abstract class MoveCommand extends CommandBase {

  protected final ClimberParentSystem subsystem;
  protected final Constraints constraints;

  protected TrapezoidProfileCommand leftTrapezoidProfileCommand;
  protected TrapezoidProfileCommand rightTrapezoidProfileCommand;

  protected Goal goal = Goal.LOW;

  // For monitoring on Shuffleboard, does nothing
  double position = 0;
    double velocity = 0;

  protected MoveCommand(ClimberParentSystem subsystem, Constraints constraints) {
    // TODO remove goal and replace with constructor parameter
    this.subsystem = subsystem;
    this.constraints = constraints;

    this.leftTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, goal.state, new State(subsystem.leftPosition(), 0)),
            this::leftPID);
    this.rightTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, goal.state, new State(subsystem.rightPosition(), 0)),
            this::rightPID);

    addRequirements(subsystem);

    Shuffleboard.getTab(subsystem.getName()).add(this);
    }

  public void toggleGoal() {
    if (goal == Goal.LOW) goal = Goal.HIGH;
    else goal = Goal.LOW;

    leftTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, goal.state, new State(subsystem.leftPosition(), 0)),
            this::leftPID);
    rightTrapezoidProfileCommand =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(constraints, goal.state, new State(subsystem.rightPosition(), 0)),
            this::rightPID);

    leftTrapezoidProfileCommand.initialize();
    rightTrapezoidProfileCommand.initialize();
  }

    @Override
    public void initialize() {
    leftTrapezoidProfileCommand.initialize();
    rightTrapezoidProfileCommand.initialize();
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
    }

  protected void leftPID(State state) {
        position = state.position;
        velocity = state.velocity;

    subsystem.leftPID(state);
    }

  protected void rightPID(State state) {
    subsystem.rightPID(state);
    }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Goal", () -> goal.state.position, null);
    builder.addDoubleProperty("Position Goal", () -> position, null);
    builder.addDoubleProperty("Velocity Goal", () -> velocity, null);
  }

    enum Goal {
        LOW(25), HIGH(195);

    private final State state;

        Goal(double state) {
      this.state = new State(state, 0);
        }
    }
}
