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
//    public JoystickButton turretHomeButton;
//    //public JoystickButton searchTargetButton;
//    public JoystickButton visionTestButton;
//    public JoystickButton lockOnTargetButton;
//    public JoystickButton intakeButton;
//    public JoystickButton turretLeftButton;
//    public JoystickButton turretRightButton;

//    public DpadButton shooterIncreaseButton;
//    public DpadButton shooterDecreaseButton;

    /**
     * Initialize SECONDARY JOYSTICK BUTTONS
     */
//    public Trigger shootTrigger;
//    public Trigger autoLineShootTrigger;



    private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick){
        precisionButton = new JoystickButton(driveJoystick, StormXboxController.stickRightButton);
//        reverseButton =   new JoystickButton(driveJoystick, StormXboxController.stickLeftButton);

        // This is a test
        reverseButton =   new JoystickButton(driveJoystick, StormXboxController.AButton);

        //        intakeButton = new JoystickButton(driveJoystick, StormXboxController.leftButton);
//
//
//        turretHomeButton = new JoystickButton(secondaryJoystick, StormXboxController.XButton);
////        searchTargetButton = new JoystickButton(secondaryJoystick, StormXboxController.BButton);
//        visionTestButton = new JoystickButton(secondaryJoystick, StormXboxController.BButton);
//        lockOnTargetButton = new JoystickButton(secondaryJoystick, StormXboxController.YButton);  // Y
//        turretLeftButton = new JoystickButton(secondaryJoystick,StormXboxController.leftButton);
//        turretRightButton = new JoystickButton(secondaryJoystick,StormXboxController.rightButton);
//
//        shooterIncreaseButton = new DpadButton(secondaryJoystick,DpadButton.Direction.UP);
//        shooterDecreaseButton = new DpadButton(secondaryJoystick,DpadButton.Direction.DOWN);
//
//
//        shootTrigger = new Trigger(() -> secondaryJoystick.getRawAxis(StormXboxController.rightTrigger) >= 0.85);
//        autoLineShootTrigger = new Trigger(() -> secondaryJoystick.getRawAxis(StormXboxController.leftTrigger) >= 0.85);
    }
}
