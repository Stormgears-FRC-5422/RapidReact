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

    public final JoystickButton autoDriveTestButton;

    public final JoystickButton selectIntakeButton;
    public final JoystickButton selectFeederButton;
    public final JoystickButton selectShooterButton;

    public final JoystickButton loadButton;
    public final JoystickButton shootButton;
    public final JoystickButton toggleShootingHeightButton;

    /**
     * Initialize SECONDARY JOYSTICK BUTTONS
     */

    private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick) {
        precisionButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);
        reverseButton = new JoystickButton(driveJoystick, StormXboxController.AButton);

        navXAlignButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);

        autoDriveTestButton = new JoystickButton(driveJoystick, StormXboxController.littleLeftButton);

        shootButton = new JoystickButton(driveJoystick, StormXboxController.rightBumper);
        loadButton = new JoystickButton(driveJoystick, StormXboxController.leftBumper);

        toggleShootingHeightButton = new JoystickButton(driveJoystick, StormXboxController.stickLeftButton);

        selectIntakeButton = new JoystickButton(secondaryJoystick, StormXboxController.BButton);
        selectFeederButton = new JoystickButton(secondaryJoystick, StormXboxController.XButton);
        selectShooterButton = new JoystickButton(secondaryJoystick, StormXboxController.YButton);
    }


}
