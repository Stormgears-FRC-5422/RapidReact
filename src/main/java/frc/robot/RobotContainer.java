// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestDrive;
import frc.robot.commands.drive.SlewDrive;
import frc.robot.commands.drive.TankDrive;
//import frc.robot.commands.TestDrive;
import frc.robot.subsystems.SafeDrive;
import frc.robot.subsystems.SparkDrive;
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
  private frc.robot.subsystems.SparkDrive drive;
//  private Intake intake;
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
      //drive = new SafeDrive();
      drive = new SparkDrive();
      // Eventually
      // if (StormMotor.motorType() == StormMotorType.TALON) drive = new TalonDrive();
      // else drive = new SparkDrive();
    }
/*
    if(Constants.useIntake){
      intake = new Intake();
    }

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
     // buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
     // buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
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
//            drive.setDefaultCommand(new DiagnosticDrive(drive, ()-> driveJoystick.getRawAxis(1), ()-> driveJoystick.getRawAxis(0),
//                                                               ()-> driveJoystick.getRawAxis(5), ()-> driveJoystick.getRawAxis(4)));
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

    if(Robot.useControlPanel) {
      controlPanel.setDefaultCommand(new MovePanel(controlPanel, driveJoystick::getAisPressed, driveJoystick::getBisPressed));
    }

    if (Robot.useTurret) {
      //    turret.setDefaultCommand(new TurnTurret(turret, secondaryJoystick, StormProp.getNumber("turretSpeed", 0.1)));
      turret.setDefaultCommand(new TurnTurret(turret, secondaryJoystick,0));
    }
*/
  }

  public void runTeleopInit() {
//    TurretGoHome turretGoHome = null;
//    if (turret.isInitialized())
//      System.out.println("Turret has been initialized");
//    else
//    {
//      turretGoHome = new TurretGoHome(turret, secondaryJoystick);
//      turretGoHome.schedule();
//      System.out.println("Turret homing is scheduled");
//    }
  }

  // Run during autonomous init phase
  public SequentialCommandGroup getAutonomousCommand() {
    return null;
//    SequentialCommandGroup autoSequence = new SequentialCommandGroup();
//
//    // Home turret if needed
//    if (Robot.useTurret && !turret.isInitialized())
//      autoSequence.addCommands(new TurretGoHome(turret,secondaryJoystick));
//
//    // Search target and lock on target
//    if (Robot.useTurret) {
//      autoSequence.addCommands(new SearchTarget(turret, vision, secondaryJoystick));
//      autoSequence.addCommands(new LockOnTarget(turret, vision, true));
//    }
//
//    // Auto shoot
//    if (Robot.useShooter && Robot.useHopper)
//      autoSequence.addCommands(new AutoShoot(shooter, hopper));
//
//    // Drive from the initiation line
//    if (Robot.useDrive)
//      autoSequence.addCommands(new CrossAutoLine(drive));
//
//    return autoSequence;
  }

  public SparkDrive getDrive() {
    return drive;
  }

}
