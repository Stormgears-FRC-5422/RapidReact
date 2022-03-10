package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;
import frc.utils.LRPair;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.*;

public class TestClimber extends CommandBase {
    private Climber climber;
    private Pivot pivot;
    private StormXboxController joystick;

    public TestClimber(Climber climber, Pivot pivot, StormXboxController joystick) {
        System.out.println("TestClimber()");
        this.climber = climber;
        this.pivot = pivot;
        this.joystick = joystick;

        this.addRequirements(climber,pivot);
    }

    @Override
    public void initialize() {
        System.out.println("TestClimber.initialize()");
    }

    @Override
    public void execute() {
        LRPair climberPair = new LRPair(joystick.getLeftJoystickY() * kClimberSpeed,
                                        joystick.getLeftJoystickY() * kClimberSpeed);
        System.out.println("Current: " + climber.getCurrent() + "  Left: " + climberPair.left + "  Right: " + climberPair.right);

        climber.setSpeed(climberPair);

        LRPair pivotPair = new LRPair(joystick.getRightJoystickY() * kPivotSpeed,
                joystick.getRightJoystickY() * kPivotSpeed);
        System.out.println("Current: " + pivot.getCurrent() + "  Left: " + pivotPair.left + "  Right: " + pivotPair.right);
        pivot.setSpeed(pivotPair);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        pivot.stop();
    }

}
