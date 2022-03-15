package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.joysticks.StormXboxController;

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

    public final JoystickButton loadButton;
    public final JoystickButton shootButton;
    public final JoystickButton toggleShootingHeightButton;

    /**
     * Initialize SECONDARY JOYSTICK BUTTONS
     */
    public final JoystickButton selectIntakeButton;
    public final JoystickButton selectFeederButton;
    public final JoystickButton selectShooterButton;

    public final JoystickButton trapezoidClimber;
    public final JoystickButton toggleClimber;

    public final JoystickButton trapezoidPivot;
    public final JoystickButton togglePivot;

//    public final JoystickButton climbLeftUpButton;
//    public final JoystickButton climbLeftDownButton;
//    public final JoystickButton climbRightUpButton;
//    public final JoystickButton climbRightDownButton;

    private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick) {
        precisionButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);
        reverseButton = new JoystickButton(driveJoystick, StormXboxController.AButton);

        navXAlignButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);

        shootButton = new JoystickButton(driveJoystick, StormXboxController.rightBumper);
        loadButton = new JoystickButton(driveJoystick, StormXboxController.leftBumper);

        toggleShootingHeightButton = new JoystickButton(driveJoystick, StormXboxController.stickLeftButton);

        // Put Secondary Joystick settings here
        selectIntakeButton = new JoystickButton(secondaryJoystick, StormXboxController.BButton);
        selectFeederButton = new JoystickButton(secondaryJoystick, StormXboxController.XButton);
        selectShooterButton = new JoystickButton(secondaryJoystick, StormXboxController.YButton);

        trapezoidClimber = new JoystickButton(secondaryJoystick, StormXboxController.littleLeftButton);
        toggleClimber = new JoystickButton(secondaryJoystick, StormXboxController.littleRightButton);

        trapezoidPivot = new JoystickButton(secondaryJoystick, StormXboxController.stickLeftButton);
        togglePivot = new JoystickButton(secondaryJoystick, StormXboxController.stickRightButton);

    }

}
