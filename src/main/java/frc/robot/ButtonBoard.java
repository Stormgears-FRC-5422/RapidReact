package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.utils.joysticks.DpadButton;
import frc.utils.joysticks.StormXboxController;

public class ButtonBoard {
    private static ButtonBoard instance;

    public static ButtonBoard getInstance(GenericHID driveJoystick, GenericHID secondaryJoystick){
        if(instance == null) instance = new ButtonBoard(driveJoystick, secondaryJoystick);
        return instance;
    }

    /**
     * Initialize DRIVE JOYSTICK BUTTONS
     */
    public JoystickButton precisionButton;
    public JoystickButton reverseButton;

    public JoystickButton selectIntakeButton;
    public JoystickButton selectFeederButton;
    public JoystickButton selectShooterButton;

    /**
     * Initialize SECONDARY JOYSTICK BUTTONS
     */

    private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick){
        precisionButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);
        reverseButton =   new JoystickButton(driveJoystick, StormXboxController.AButton);

        selectIntakeButton = new JoystickButton(driveJoystick, StormXboxController.BButton);
        selectFeederButton = new JoystickButton(driveJoystick, StormXboxController.XButton);
        selectShooterButton = new JoystickButton(driveJoystick, StormXboxController.YButton);
    }
}
