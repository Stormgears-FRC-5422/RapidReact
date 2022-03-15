package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.Climber;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class PositionClimber extends CommandBase {

    enum Goal {
        LOW(20), HIGH(200);

        private final State state;

        Goal(double state){
            this.state = new State(state, 0);
        }
    }

    private final Climber climber;

    private TrapezoidProfileCommand leftClimberController;
    private TrapezoidProfileCommand rightClimberController;

    private final Constraints constraints = new Constraints(-150, -75);

    private Goal goal = Goal.HIGH;

    public PositionClimber(Climber climber) {
        this.climber = climber;

        this.leftClimberController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.leftPosition(), 0)), this::leftPID);
        this.rightClimberController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.rightPosition(), 0)), this::rightPID);

        this.addRequirements(climber);
    }

    public void toggleGoal(){
        if (goal == Goal.LOW) goal = Goal.HIGH;
        else goal = Goal.LOW;

        leftClimberController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.leftPosition(), 0)), this::leftPID);
        rightClimberController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.rightPosition(), 0)), this::rightPID);
    }

    @Override
    public void initialize() {
        leftClimberController.initialize();
        rightClimberController.initialize();
    }

    @Override
    public void execute() {
        leftClimberController.execute();
        rightClimberController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        leftClimberController.end(interrupted);
        rightClimberController.end(interrupted);
    }

    private void leftPID(State state){
        climber.leftPID(state);
    }

    private void rightPID(State state){
        climber.rightPID(state);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal", () -> goal.state.position, null);
    }
}
