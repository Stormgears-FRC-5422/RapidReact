package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static frc.utils.joysticks.StormXboxController.*;

public class ButtonBoard {
  private static ButtonBoard instance;
  /** Initialize DRIVE JOYSTICK BUTTONS */
  public final JoystickButton precisionButton;
  public final JoystickButton loadButton;
  public final JoystickButton shootButton;
  public final JoystickButton toggleShootingHeightButton;
  /** Initialize SECONDARY JOYSTICK BUTTONS */
  //  public final JoystickButton trapezoidClimber;

  //  public final JoystickButton trapezoidPivot;
  public final POVButton homeClimbing;

  public final POVButton homeClimberButton;
  public final POVButton homePivotButton;
  public final JoystickButton manualClimberButton;
  public final POVButton coordinatingClimberButton;
  public final JoystickButton climberUP;
  public final JoystickButton climberDown;
  public final JoystickButton backBend;
  public final JoystickButton autoClimb;
  public final JoystickButton reverseButton;
  public final POVButton liftIntakeButton;
  public final JoystickButton driveWithVisionButton;
  public final JoystickButton shootWithVisionButton;

  private ButtonBoard(GenericHID driveJoystick, GenericHID secondaryJoystick) {
    // ********************************
    // Put Drive Joystick settings here
    // ********************************
    precisionButton = new JoystickButton(driveJoystick, leftBumper);
    climberUP = new JoystickButton(driveJoystick, YButton);
    climberDown = new JoystickButton(driveJoystick, AButton);
    autoClimb = new JoystickButton(driveJoystick, BButton);
    backBend = new JoystickButton(driveJoystick, XButton);
    reverseButton = new JoystickButton(driveJoystick, littleLeftButton);
    // ********************************
    // Put Secondary Joystick settings here
    // ********************************
    driveWithVisionButton = new JoystickButton(secondaryJoystick, YButton);
    shootWithVisionButton = new JoystickButton(secondaryJoystick, rightBumper);
    shootButton = new JoystickButton(secondaryJoystick, stickLeftButton);
    loadButton = new JoystickButton(secondaryJoystick, leftBumper);
    toggleShootingHeightButton = new JoystickButton(secondaryJoystick, stickRightButton);
    manualClimberButton = new JoystickButton(secondaryJoystick, BButton);
    coordinatingClimberButton = new POVButton(secondaryJoystick, 0);
    liftIntakeButton = new POVButton(secondaryJoystick, 90);
    homeClimbing = new POVButton(secondaryJoystick, 180);


    homePivotButton = new POVButton(secondaryJoystick, 180);
    homeClimberButton = new POVButton(secondaryJoystick, 180);
  }

  public static ButtonBoard getInstance(GenericHID driveJoystick, GenericHID secondaryJoystick) {
    if (instance == null) instance = new ButtonBoard(driveJoystick, secondaryJoystick);
    return instance;
  }
}
