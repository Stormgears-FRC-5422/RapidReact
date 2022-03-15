package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.kClimberSpeed;
import static frc.robot.Constants.kPivotSpeed;

public class TestClimber extends CommandBase {
    private Climber climber;
    private Pivot pivot;
    private StormXboxController joystick;

    public TestClimber(Climber climber, Pivot pivot, StormXboxController joystick) {
        System.out.println("TestClimber()");
        this.climber = climber;
        this.pivot = pivot;
        this.joystick = joystick;

        addRequirements(climber,pivot);
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

        LRSpeeds climberSpeeds = new LRSpeeds(joystick.getLeftJoystickY() * kClimberSpeed,
                                        joystick.getLeftJoystickY() * kClimberSpeed);
        LRSpeeds pivotSpeeds = new LRSpeeds(joystick.getRightJoystickY() * kPivotSpeed,
                joystick.getRightJoystickY() * kPivotSpeed);

        // Move only the one on the side with the bumper held
        if (joystick.getLeftBumperIsHeld()) {
            climberSpeeds.disableRight();
            pivotSpeeds.disableRight();
        } else if (joystick.getRightBumperIsHeld()) {
            climberSpeeds.disableLeft();
            pivotSpeeds.disableLeft();
        }

        climber.setSpeed(climberSpeeds);
        pivot.setSpeed(pivotSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        pivot.stop();
    }

}
