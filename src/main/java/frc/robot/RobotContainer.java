// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TestDrive;
import frc.robot.commands.intake.Load;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.TestIntake;
import frc.robot.subsystems.ballHandler.DiagnosticIntake;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.drive.TalonDrive;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

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
  private DiagnosticIntake diagnosticIntake;

  private Shooter shooter;
  private Feeder feeder;
  private Intake intake;

  private Load load;
  private Shoot shoot;

  /**
   * Joysticks and buttons
   */
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
    if (Constants.useIntake && Constants.useFeeder) load = new Load(intake, feeder);
    if (Constants.useShooter && Constants.useFeeder && Constants.useIntake) shoot = new Shoot(feeder, shooter, intake);
  }

  private void initSubsystems() {
    if (Constants.useDrive) {
      switch (Constants.MOTOR_TYPE) {
        case "Spark":
          drive = new SparkDrive();
          break;
        case "Talon":
          drive = new TalonDrive();
          break;
        default:
      }
    }

    if (Constants.diagnostic) diagnosticIntake = new DiagnosticIntake();
    else {
      if (Constants.useShooter) shooter = new Shooter();
      if (Constants.useFeeder) feeder = new Feeder();
      if (Constants.useIntake) intake = new Intake();
    }
  }

  private void configureButtonBindings() {
    if (Constants.useDrive) {
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
    }
    if (Constants.diagnostic) {
      if (Constants.useIntake) buttonBoard.selectIntakeButton.whenPressed(diagnosticIntake::setModeIntake);
      if (Constants.useFeeder) buttonBoard.selectFeederButton.whenPressed(diagnosticIntake::setModeFeeder);
      if (Constants.useShooter) buttonBoard.selectShooterButton.whenPressed(diagnosticIntake::setModeShooter);
    } else {
      if (Constants.useIntake && Constants.useFeeder) buttonBoard.loadButton.whileHeld(load);
      if (Constants.useShooter && Constants.useFeeder) buttonBoard.shootButton.whileHeld(shoot);
    }
  }

  private void configureDefaultCommands() {
    if (Constants.useDrive) {
      drive.setDefaultCommand(new TestDrive(drive, driveJoystick));
    }

    if (Constants.diagnostic) {
      diagnosticIntake.setDefaultCommand(new TestIntake(diagnosticIntake, driveJoystick));
    }

  }

  public void runTeleopInit() {
  }

  // Run during autonomous init phase
  public SequentialCommandGroup getAutonomousCommand() {
    return null;
  }

  public StormDrive getDrive() {
    return drive;
  }

}
