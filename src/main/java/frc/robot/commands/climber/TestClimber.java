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
        if (joystick.getAisPressed()) {
            climber.zero();
            pivot.zero();
            return;
        }

        if (joystick.getXisPressed()) {
            climber.disableLimits();
            pivot.disableLimits();
        }

        if (joystick.getYisPressed()) {
            climber.enableLimits();
            pivot.enableLimits();
        }

        LRPair climberPair = new LRPair(joystick.getLeftJoystickY() * kClimberSpeed,
                                        joystick.getLeftJoystickY() * kClimberSpeed);
        LRPair pivotPair = new LRPair(joystick.getRightJoystickY() * kPivotSpeed,
                joystick.getRightJoystickY() * kPivotSpeed);

        // Move only the one on the side with the bumper held
        if (joystick.getLeftBumperIsHeld()) {
            climberPair.right = 0;
            pivotPair.right = 0;
        } else if (joystick.getRightBumperIsHeld()) {
            climberPair.left = 0;
            pivotPair.left = 0;
        }

        climber.setSpeed(climberPair);
        pivot.setSpeed(pivotPair);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        pivot.stop();
    }

}
