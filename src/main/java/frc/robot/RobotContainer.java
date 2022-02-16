// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TestDrive;
//import frc.robot.commands.drive.TestDrive;
import frc.robot.commands.intake.TestIntake;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.drive.TalonDrive;
import frc.robot.subsystems.intake.Intake;
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
  //private frc.robot.subsystems.SafeDrive drive;
  private frc.utils.drive.StormDrive drive;
  private Intake intake;

//  private Shooter shooter;
//  private Climber climber;
//  private StatusLights statusLights;

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
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void initSubsystems(){
    if(Constants.useDrive){
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

    if(Constants.useIntake){
      intake = new Intake();
    } else System.out.println("Intake disabled");

/*
    if(Constants.useShooter){
      shooter = new Shooter();
    }

    if(Constants.useControlPanel){
      controlPanel = new ControlPanel();
    }

    if(Constants.useTurret) {
      turret = new Turret();
    }

    if(Constants.useVision){
      vision = new Vision();
    }

    if (Constants.useClimber) {
      climber = new Climber();
    }

    if (Constants.useStatusLights) {
      statusLights = StatusLights.getInstance(); //just to initialize
      statusLights.setAscending(5);
    }
  */
  }

  private void configureButtonBindings() {
    if(Constants.useDrive){
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
    }

    if(Constants.useIntake) {
      buttonBoard.selectIntakeButton.whenPressed(intake::setModeIntake);
      buttonBoard.selectFeederButton.whenPressed(intake::setModeFeeder);
      buttonBoard.selectShooterButton.whenPressed(intake::setModeShooter);
    }

//    if(Constants.useShooter){
//      boolean useVelocity = StormProp.getBoolean("shooterUseVelocity", false);
//      buttonBoard.shootTrigger.whileActiveContinuous(new ShootVariable(shooter, vision, () -> 0.0, useVelocity));
//      buttonBoard.autoLineShootTrigger.whileActiveContinuous(new ShootVariable(shooter, vision, ()-> 1.0, useVelocity));
//      buttonBoard.shooterDecreaseButton.whenPressed(new AdjustShooterPower(shooter,AdjustShooterPower.Direction.DOWN));
//      buttonBoard.shooterIncreaseButton.whenPressed(new AdjustShooterPower(shooter,AdjustShooterPower.Direction.UP));
//    }

  }

  private void configureDefaultCommands() {
    if (Constants.useDrive) {
      System.out.println("Just created the Drive object");
      drive.setDefaultCommand(new TestDrive(drive, driveJoystick));
    }

    if(Constants.useIntake) {
      intake.setDefaultCommand(new TestIntake(intake, driveJoystick));
    }

    /*
    if(Robot.useHopper) {
      hopper.setDefaultCommand(new ReloadHopper(hopper, driveJoystick::getLeftJoystickY));
    }

    if(Robot.useIntake) {
      intake.setDefaultCommand(new IntakeBall(intake, () -> -driveJoystick.getLeftJoystickY()));
    }

    if(Robot.useShooter) {
      shooter.setDefaultCommand(new SpinIndexRoller(shooter, driveJoystick::getLeftJoystickY));
    }

    // temp controls!
    if(Robot.useClimber) {
      climber.setDefaultCommand(new Climb(climber, () -> driveJoystick.getLeftJoystickY(), () -> driveJoystick.getRightJoystickY()));
    }

    }
*/
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
