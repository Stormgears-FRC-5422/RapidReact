package frc.robot;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.autonomous.DoubleBallAuto;
import frc.robot.commands.ballHandler.*;
import frc.robot.commands.climber.ManualClimber;
import frc.robot.commands.climber.automatedMovement.ClimbingSequences;
import frc.robot.commands.climber.home.Home;
import frc.robot.commands.climber.home.HomeClimbingSystem;
import frc.robot.commands.drive.DriveWithVision;
import frc.robot.commands.drive.SlewDrive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.drive.TalonDrive;
import frc.robot.subsystems.sensors.NavX;
import frc.robot.subsystems.sensors.Vision;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

// TODO Verify Intake comes down in auto, verify all buttons work
public class RobotContainer {

  private final boolean useDriveJoystick;
  private final boolean useSecondaryJoystick;

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  // Declare subsystems - initialize below
  private Lights lights;
  @Log private StormDrive drive;
  @Log private NavX navX;
  @Log private Shooter shooter;
  private Feeder feeder;
  private Intake intake;
  private Vision vision;
  @Log.Include private Climber climber;
  @Log.Include private Pivot pivot;

  private SlewDrive slewDrive;
  private DriveWithVision driveWithVision;
  @Log private Shoot shoot;
  private ShootWithVision shootWithVision;
  private DoubleArrayLogEntry shooterDistanceRPSLog;
  private LiftIntake liftIntake;
  private Load load;
  private Reverse reverse;
  private ClimbingSequences climberSequences;
  private CommandBase homingSequence;
  private ManualClimber manualClimber;
  private ManualClimber manualPivot;

  private CommandBase autonomous;

  public RobotContainer(DoubleArrayLogEntry shooterDistanceRPSLog) {
    if (shooterDistanceRPSLog != null) this.shooterDistanceRPSLog = shooterDistanceRPSLog;
    driveJoystick = new StormXboxController(0);
    secondaryJoystick = new StormXboxController(1);
    buttonBoard = ButtonBoard.getInstance(driveJoystick, secondaryJoystick);

    // TODO - make this configurable (want to keep true for competition)
    useDriveJoystick = true; // (kUseController && kUseJoystick0 && driveJoystick.isConnected());
    useSecondaryJoystick =
        true; // (kUseController && kUseJoystick1 && secondaryJoystick.isConnected());
    System.out.println(
        "useDriveJoystick is "
            + useDriveJoystick
            + ", useSecondaryJoystick is "
            + useSecondaryJoystick);

    initSubsystems();
    initCommands();
    configureDefaultCommands();
    configureButtonBindings();
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
    if (kUseClimber) climber = new Climber();
    if (kUsePivot) pivot = new Pivot();

    if (kUseFeeder) feeder = new Feeder();
    if (!kDiagnostic) {
      if (kUseShooter) shooter = new Shooter();
      if (kUseIntake) intake = new Intake();
      if (kUseLights) lights = new Lights();
    }
    if (kUseVision) vision = new Vision();
  }

  public StormDrive getDrive() {
    return drive;
  }

  public CommandBase getHomingSequence() {
    return homingSequence == null ? new InstantCommand() : homingSequence;
  }

  public void configureDefaultCommands() {
    if (kUseShooter && kUseFeeder) {
      shooter.setDefaultCommand(new RunIdleShooter(shooter, feeder::getAbsoluteLimit));
    }
    //    if (useDriveJoystick) {}
    //    if (useSecondaryJoystick) {}
  }

  private void initCommands() {
    if (kUseDrive)
      slewDrive =
          new SlewDrive(drive, driveJoystick::getTriggerSpeed, driveJoystick::getLeftJoystickX);

    if (!kDiagnostic) {
      if (kUseFeeder) liftIntake = new LiftIntake(feeder, secondaryJoystick);
      if (kUseIntake && kUseFeeder) {
        reverse = new Reverse(intake, feeder);
        load = new Load(intake, feeder);
      }
      if (kUseShooter && kUseFeeder && kUseLights) shoot = new Shoot(feeder, shooter, lights);
      else if (kUseShooter && kUseFeeder) shoot = new Shoot(feeder, shooter, null);
    }

    if (kUsePivot && kUseClimber) {
      climberSequences = new ClimbingSequences(climber, pivot);
      homingSequence = new HomeClimbingSystem(climber, pivot);
    } else if (kUsePivot) homingSequence = new Home(pivot);
    else if (kUseClimber) homingSequence = new Home(climber);

    if (kUseClimber)
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
    if (kUsePivot)
      manualPivot =
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getRightJoystickY);

    if (kUseVision) {
      if (kUseDrive)
        driveWithVision =
            new DriveWithVision(
                drive, driveJoystick::getTriggerSpeed, driveJoystick::getLeftJoystickX, vision);
      if (kUseFeeder && kUseShooter)
        shootWithVision =
            new ShootWithVision(
                shooter, shoot, vision::hasTarget, vision::getDistance, shooterDistanceRPSLog);
    }
    if (kUseFeeder && kUsePivot && kUseIntake && kUseDrive && kUseVision)
      autonomous = new DoubleBallAuto(load, drive, vision, shootWithVision);
  }

  public void configureButtonBindings() {
    if (!kUseController) return;

    if (useDriveJoystick) {
      System.out.println("Setting up drive joystick commands");
      if (kUseDrive) {
        System.out.println("... drive");
        buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
      }

      if (!kDiagnostic) {
        if (kUseIntake && kUseFeeder) {
          // TODO Does reverse work?
          buttonBoard.reverseButton.whileHeld(reverse);
          // TODO Does lift intake work?
          buttonBoard.liftIntakeButton.whenPressed(liftIntake);
        }
        if (kUseShooter && kUseFeeder)
          buttonBoard.toggleShootingHeightButton.whenPressed(shoot::toggleMode);
      }
      if (kUsePivot && kUseClimber) {
        buttonBoard.climberUP.whenPressed(climberSequences.highestClimber);
        buttonBoard.climberDown.whenPressed(climberSequences.chinUp);
        buttonBoard.autoClimb.whenPressed(climberSequences.autoClimb);
        buttonBoard.backBend.whenPressed(climberSequences.backbend);
      }
    }

    if (useSecondaryJoystick) {
      System.out.println("Setting up secondary joystick commands");
      if (kUseFeeder && kUseIntake) buttonBoard.loadButton.whileHeld(load);
      if (kUseFeeder && kUseShooter) {
        buttonBoard.shootButton.whileHeld(shoot);
        if (kUseVision) {
          buttonBoard.shootWithVisionButton.whileHeld(shootWithVision);
          buttonBoard.driveWithVisionButton.whileHeld(driveWithVision);
        }
      }
      if (kUsePivot || kUseClimber) buttonBoard.homeClimbing.whenPressed(getHomingSequence());
      if (kUsePivot && kUseClimber) {
        buttonBoard.coordinatingClimberButton.whenPressed(climberSequences.coordinatingClimber);
        buttonBoard.manualClimberButton.whenPressed(
            new ParallelCommandGroup(manualClimber, manualPivot).withName("ManualMode"));
      } else if (kUsePivot) buttonBoard.manualClimberButton.whenPressed(manualPivot);
      else if (kUseClimber) buttonBoard.manualClimberButton.whenPressed(manualClimber);
    }
  }

  public CommandBase getAutonomous() {
    return autonomous;
  }

  @Config(name = "ON FOR TWO BALLS (pause)", tabName = "Autonomous", defaultValueBoolean = true)
  public void setAutonomous(boolean twoBall) {
    if (twoBall) {
      autonomous = new DoubleBallAuto(load, drive, vision, shootWithVision);
      System.out.println("Two Ball");
    } else {
      autonomous = new Autonomous(load, shoot, drive);
      System.out.println("One ball");
    }
  }

  public void setDrive() {
    if (kUseDrive) {
      drive.setDefaultCommand(slewDrive);
      slewDrive.schedule(true);
    }
  }

  public void feederCoast() {
    if (kUseFeeder) feeder.setCoast();
  }
}
