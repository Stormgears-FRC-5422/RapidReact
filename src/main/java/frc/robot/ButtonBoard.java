package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.utils.joysticks.StormXboxController.*;

import static frc.robot.Constants.*;

public class ButtonBoard {
    private static ButtonBoard instance;

    public static ButtonBoard getInstance(GenericHID driveJoystick, GenericHID secondaryJoystick) {
        if (instance == null) instance = new ButtonBoard(driveJoystick, secondaryJoystick);
        return instance;
    }

    /**
     * Initialize DRIVE JOYSTICK BUTTONS
     */
    public final JoystickButton precisionButton;
    public final JoystickButton reverseButton;

    public final JoystickButton navXAlignButton;

    public final JoystickButton autoDriveTestButton;
    public final JoystickButton autoDriveTestReverseButton;

    public final JoystickButton loadButton;
    public final JoystickButton shootButton;
    public final JoystickButton toggleShootingHeightButton;

    /**
     * Initialize SECONDARY JOYSTICK BUTTONS
     */

    public final JoystickButton trapezoidClimber;
    public final JoystickButton trapezoidPivot;

    public final JoystickButton homeClimbing;
    public final JoystickButton homeClimberButton;
    public final JoystickButton homePivotButton;

    public final Button manualClimberButton;
//    public final JoystickButton selectIntakeButton;
//    public final JoystickButton selectFeederButton;
//    public final JoystickButton selectShooterButton;
//    public final JoystickButton climbLeftUpButton;
//    public final JoystickButton climbLeftDownButton;
//    public final JoystickButton climbRightUpButton;
//    public final JoystickButton climbRightDownButton;

    private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick) {
    // ********************************
    // Put Drive Joystick settings here
    // ********************************
    precisionButton = new JoystickButton(driveJoystick, stickRightButton);
    reverseButton = new JoystickButton(driveJoystick, AButton);
    navXAlignButton = new JoystickButton(driveJoystick, stickRightButton);
    autoDriveTestButton = new JoystickButton(driveJoystick, littleLeftButton);
    autoDriveTestReverseButton = new JoystickButton(driveJoystick, littleRightButton);

    shootButton = new JoystickButton(driveJoystick, rightBumper);
    loadButton = new JoystickButton(driveJoystick, leftBumper);
    toggleShootingHeightButton = new JoystickButton(driveJoystick, stickLeftButton);

    // ********************************
    // Put Secondary Joystick settings here
    // ********************************
    homePivotButton = new JoystickButton(secondaryJoystick, stickRightButton);
    homeClimberButton = new JoystickButton(secondaryJoystick, stickLeftButton);
    homeClimbing = new JoystickButton(secondaryJoystick, stickLeftButton);

    trapezoidClimber = new JoystickButton(secondaryJoystick, littleLeftButton);
    trapezoidPivot = new JoystickButton(secondaryJoystick, littleRightButton);

    manualClimberButton = new JoystickButton(secondaryJoystick, BButton);
    }

}
