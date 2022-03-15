package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimberParentSystem;
import frc.robot.subsystems.climber.Pivot;

public class PositionPivot extends MoveCommand {

    private final Pivot pivot;
    protected Goal goal = Goal.HIGH;

    public PositionPivot(Pivot pivot) {
        this.pivot = pivot;

        this.leftController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(pivot.leftPosition(), 0)), this::leftPID);
        this.rightController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(pivot.rightPosition(), 0)), this::rightPID);

        this.addRequirements(pivot);

        Shuffleboard.getTab("Pivot").add(this);
    }

    @Override
    public ClimberParentSystem subsystem() {
        return pivot;
    }

    @Override
    public void toggleGoal() {
        if (goal == Goal.LOW) goal = Goal.HIGH;
        else goal = Goal.LOW;

        leftController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(subsystem().leftPosition(), 0)), this::leftPID);
        rightController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(subsystem().rightPosition(), 0)), this::rightPID);

        leftController.initialize();
        rightController.initialize();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal", () -> goal.state.position, null);
        builder.addDoubleProperty("Position Goal", () -> position, null);
        builder.addDoubleProperty("Velocity Goal", () -> velocity, null);
    }

    enum Goal {
        LOW(25), HIGH(195);

        public final TrapezoidProfile.State state;

        Goal(double state) {
            this.state = new TrapezoidProfile.State(state, 0);
        }
    }
}
