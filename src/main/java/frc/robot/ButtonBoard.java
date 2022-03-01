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

        selectIntakeButton = new JoystickButton(driveJoystick, StormXboxController.BButton);
        selectFeederButton = new JoystickButton(driveJoystick, StormXboxController.XButton);
        selectShooterButton = new JoystickButton(driveJoystick, StormXboxController.YButton);

        shootButton = new JoystickButton(driveJoystick, StormXboxController.rightButton);
        loadButton = new JoystickButton(driveJoystick, StormXboxController.leftButton);

        toggleShootingHeightButton = new JoystickButton(driveJoystick, StormXboxController.stickLeftButton);
    }
}
