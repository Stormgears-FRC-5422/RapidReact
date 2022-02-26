package frc.robot;

import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.ballHandler.TestIntake;
import frc.robot.commands.drive.TestDrive;
import frc.robot.commands.navX.NavXAlign;
import frc.robot.subsystems.ballHandler.DiagnosticIntake;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.drive.TalonDrive;
import frc.robot.subsystems.sensors.NavX;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * Declare subsystems - initialize below
   */
  private StormDrive drive;
  private NavX navX;
  private DiagnosticIntake diagnosticIntake;

  private Shooter shooter;
  private Feeder feeder;
  private Intake intake;

  private Load load;
  private Shoot shoot;
  private NavXAlign navXAlign;

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  public RobotContainer() {
    driveJoystick = new StormXboxController(0);
    secondaryJoystick = new StormXboxController(1);
    buttonBoard = ButtonBoard.getInstance(driveJoystick, secondaryJoystick);

    initSubsystems();
    initCommands();
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void initCommands() {
    if (!kDiagnostic && kUseFeeder) {
      if (kUseIntake) load = new Load(intake, feeder);
      if (kUseShooter) shoot = new Shoot(feeder, shooter);
    }
    if (kUseNavX) navXAlign = new NavXAlign(drive, navX);
  }

  private void initSubsystems() {
    if (kUseDrive) {
      switch (kMotorType) {
        case "Spark":
          drive = new SparkDrive();
          break;
        case "Talon":
          drive = new TalonDrive();
          break;
        default:
      }
    }
    if (kUseNavX) navX = new NavX();

    if (kDiagnostic) diagnosticIntake = new DiagnosticIntake();
    else {
      if (kUseShooter) shooter = new Shooter();
      if (kUseFeeder) feeder = new Feeder();
      if (kUseIntake) intake = new Intake();
    }
  }

  private void configureButtonBindings() {
    if (kUseDrive) {
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
    }
    if (kDiagnostic) {
      if (kUseIntake) buttonBoard.selectIntakeButton.whenPressed(diagnosticIntake::setModeIntake);
      if (kUseFeeder) buttonBoard.selectFeederButton.whenPressed(diagnosticIntake::setModeFeeder);
      if (kUseShooter) buttonBoard.selectShooterButton.whenPressed(diagnosticIntake::setModeShooter);
    } else {
      if (kUseIntake && kUseFeeder) buttonBoard.loadButton.whileHeld(load);
      if (kUseShooter && kUseFeeder) buttonBoard.shootButton.whileHeld(shoot);
    }
    if (kUseNavX) buttonBoard.navXAlignButton.whileHeld(navXAlign);
  }

  private void configureDefaultCommands() {
    if (kUseDrive) drive.setDefaultCommand(new TestDrive(drive, driveJoystick));
    if (kDiagnostic) diagnosticIntake.setDefaultCommand(new TestIntake(diagnosticIntake, driveJoystick));
  }

  public StormDrive getDrive() {
    return drive;
  }
}
