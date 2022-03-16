package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimberParentSystem;

public abstract class MoveCommand extends CommandBase {
    protected final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(150, 75);
    protected TrapezoidProfileCommand leftController;
    protected TrapezoidProfileCommand rightController;
  protected Goal goal = Goal.LOW;
    double position = 0;
    double velocity = 0;


    protected MoveCommand() {
    }

  public void toggleGoal() {
    if (goal == Goal.LOW) goal = Goal.HIGH;
    else goal = Goal.LOW;

    leftController =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(
                constraints, goal.state, new TrapezoidProfile.State(subsystem().leftPosition(), 0)),
            this::leftPID);
    rightController =
        new TrapezoidProfileCommand(
            new TrapezoidProfile(
                constraints,
                goal.state,
                new TrapezoidProfile.State(subsystem().rightPosition(), 0)),
            this::rightPID);

    leftController.initialize();
    rightController.initialize();
  }

    @Override
    public void initialize() {
        leftController.initialize();
        rightController.initialize();
    }

    @Override
    public void execute() {
        leftController.execute();
        rightController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        leftController.end(interrupted);
        rightController.end(interrupted);
    }

    protected void leftPID(TrapezoidProfile.State state) {
        position = state.position;
        velocity = state.velocity;

        subsystem().leftPID(state);
    }

    protected void rightPID(TrapezoidProfile.State state) {
        subsystem().rightPID(state);
    }

    @Override
    public abstract void initSendable(SendableBuilder builder);

    public abstract ClimberParentSystem subsystem();

    enum Goal {
        LOW(25), HIGH(195);

        public final TrapezoidProfile.State state;

        Goal(double state) {
            this.state = new TrapezoidProfile.State(state, 0);
        }
    }
}
