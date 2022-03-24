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
  public final JoystickButton homeClimbing;

  public final JoystickButton homeClimberButton;
  public final JoystickButton homePivotButton;
  public final JoystickButton manualClimberButton;
  public final POVButton coordinatingClimberButton;
  public final JoystickButton climberUP;
  public final JoystickButton climberDown;
  public final JoystickButton pivotIN;
  public final JoystickButton pivotOut;
  public final JoystickButton reverseButton;

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
    //    reverseButton = new JoystickButton(driveJoystick, rightBumper);
    //    autoDriveTestButton = new JoystickButton(driveJoystick, littleLeftButton);
    //    autoDriveTestReverseButton = new JoystickButton(driveJoystick, littleRightButton);

    shootButton = new JoystickButton(secondaryJoystick, rightBumper);
    loadButton = new JoystickButton(secondaryJoystick, leftBumper);
    toggleShootingHeightButton = new JoystickButton(secondaryJoystick, stickRightButton);

    // ********************************
    // Put Secondary Joystick settings here
    // ********************************
    homePivotButton = new JoystickButton(secondaryJoystick, stickLeftButton);
    homeClimberButton = new JoystickButton(secondaryJoystick, stickLeftButton);
    homeClimbing = new JoystickButton(secondaryJoystick, stickLeftButton);

    //    trapezoidClimber = new JoystickButton(secondaryJoystick, littleLeftButton);
    //    trapezoidPivot = new JoystickButton(secondaryJoystick, littleRightButton);

    manualClimberButton = new JoystickButton(secondaryJoystick, BButton);
    coordinatingClimberButton = new POVButton(secondaryJoystick,0);

  }
  //    public final JoystickButton selectIntakeButton;
  //    public final JoystickButton selectFeederButton;
  //    public final JoystickButton selectShooterButton;
  //    public final JoystickButton climbLeftUpButton;
  //    public final JoystickButton climbLeftDownButton;
  //    public final JoystickButton climbRightUpButton;
  //    public final JoystickButton climbRightDownButton;

  public static ButtonBoard getInstance(GenericHID driveJoystick, GenericHID secondaryJoystick) {
    if (instance == null) instance = new ButtonBoard(driveJoystick, secondaryJoystick);
    return instance;
  }
}
