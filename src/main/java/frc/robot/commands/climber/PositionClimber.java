package frc.robot.commands.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

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

    private Climber climber;
    private StormXboxController joystick;

    private ProfiledPIDController leftClimberController;
    private ProfiledPIDController rightClimberController;

    private Constraints constraints = new Constraints(-150, -75);

    private Goal goal = Goal.HIGH;

    public PositionClimber(Climber climber, StormXboxController joystick) {
        System.out.println("TestClimber()");
        this.climber = climber;
        this.joystick = joystick;

        this.leftClimberController = new ProfiledPIDController(0,0,0, constraints);
        this.rightClimberController = new ProfiledPIDController(0,0,0, constraints);

        this.addRequirements(climber);
    }

    public void toggleGoal(){
        if (goal == Goal.LOW) goal = Goal.HIGH;
        else goal = Goal.LOW;
    }

    @Override
    public void initialize() {
        leftClimberController.reset(climber.leftPosition());
        rightClimberController.reset(climber.rightPosition());
    }

    @Override
    public void execute() {
        double left = leftClimberController.calculate(climber.leftPosition(), goal.state.position);
        double right = rightClimberController.calculate(climber.rightPosition(), goal.state.position);

        climber.setGoal(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setSpeed(new LRSpeeds(0, 0));
    }
}
