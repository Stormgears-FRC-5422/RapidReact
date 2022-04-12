package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static frc.utils.joysticks.StormXboxController.*;

public class ButtonBoard {
  private static ButtonBoard instance;
  /** Initialize DRIVE JOYSTICK BUTTONS */
  public final JoystickButton precisionButton;

  //  public final JoystickButton reverseButton;
  //  public final JoystickButton autoDriveTestButton;
  //  public final JoystickButton autoDriveTestReverseButton;
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
  public final JoystickButton pivotIN;
  public final JoystickButton pivotOut;
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
    pivotIN = new JoystickButton(driveJoystick, BButton);
    pivotOut = new JoystickButton(driveJoystick, XButton);
    reverseButton = new JoystickButton(driveJoystick, littleLeftButton);
    driveWithVisionButton = new JoystickButton(driveJoystick, stickRightButton);
    // ********************************
    // Put Secondary Joystick settings here
    // ********************************
    shootWithVisionButton = new JoystickButton(secondaryJoystick, YButton);
    shootButton = new JoystickButton(secondaryJoystick, rightBumper);
    loadButton = new JoystickButton(secondaryJoystick, leftBumper);
    toggleShootingHeightButton = new JoystickButton(secondaryJoystick, stickRightButton);
    homePivotButton = new POVButton(secondaryJoystick, 180);
    homeClimberButton = new POVButton(secondaryJoystick, 180);
    homeClimbing = new POVButton(secondaryJoystick, 180);
    manualClimberButton = new JoystickButton(secondaryJoystick, BButton);
    coordinatingClimberButton = new POVButton(secondaryJoystick,0);
    liftIntakeButton = new POVButton(secondaryJoystick, 90);
  }

  public static ButtonBoard getInstance(GenericHID driveJoystick, GenericHID secondaryJoystick) {
    if (instance == null) instance = new ButtonBoard(driveJoystick, secondaryJoystick);
    return instance;
  }
}
