package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.autonomous.DoubleBallAuto;
import frc.robot.commands.ballHandler.*;
import frc.robot.commands.climber.ManualClimber;
import frc.robot.commands.climber.hold.HoldCurrentPosition;
import frc.robot.commands.climber.hold.HoldTargetPosition;
import frc.robot.commands.climber.home.Home;
import frc.robot.commands.climber.home.HomeClimbingSystem;
import frc.robot.commands.climber.trapezoid.CoordinatingClimber;
import frc.robot.commands.climber.trapezoid.PositionClimber;
import frc.robot.commands.climber.trapezoid.PositionPivot;
import frc.robot.commands.drive.DriveWithVision;
import frc.robot.commands.drive.SlewDrive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ballHandler.DiagnosticIntake;
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
import static frc.robot.commands.climber.trapezoid.ClimbingGoal.*;
import static frc.robot.commands.climber.trapezoid.PivotGoal.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final boolean useDriveJoystick;
  private final boolean useSecondaryJoystick;
  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  @Config.Command(tabName = "Driver", name = "UP Climber TRAVERSE")
  private SequentialCommandGroup highestClimber;

  @Config.Command(tabName = "Driver", name = "Down Climber")
  private SequentialCommandGroup chinUp;

  public Feeder feeder;

  @Config.Command(tabName = "Driver", name = "Chin Up pivot")
  private SequentialCommandGroup backbendAndChinUp;
  /** Declare subsystems - initialize below */
  @Log private StormDrive drive;

  @Config.Command(tabName = "Driver", name = "back Pivot")
  private SequentialCommandGroup backbend;

  @Log private NavX navX;
  private DiagnosticIntake diagnosticIntake;
  @Log private Shooter shooter;
  private Vision vision;
  private Intake intake;
  private Lights lights;
  private TestIntake testIntake;
  private LiftIntake liftIntake;
  private Load load;
  @Log private Shoot shoot;
  private CommandBase homingSequence;
  private ManualClimber manualClimber;
  private CoordinatingClimber coordinatingClimber;
  private ManualClimber manualPivot;
  @Log private HoldCurrentPosition climberHoldCurrentPosition;
  @Log private HoldCurrentPosition pivotHoldCurrentPosition;
  private DriveWithVision driveWithVision;
  private ShootWithVision shootWithVision;

  private Reverse reverse;

  @Log.Include private Climber climber;
  @Log.Include private Pivot pivot;
  private CommandBase autonomous;
  private SlewDrive slewDrive;

  public RobotContainer() {
    driveJoystick = new StormXboxController(0);
    secondaryJoystick = new StormXboxController(1);
    buttonBoard = ButtonBoard.getInstance(driveJoystick, secondaryJoystick);

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

    if (kDiagnostic) {
      // diagnosticIntake = new DiagnosticIntake();
      if (kUseFeeder) feeder = new Feeder();
    } else {
      if (kUseShooter) shooter = new Shooter();
      if (kUseFeeder) feeder = new Feeder();
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
    if (useDriveJoystick) {}
    if (useSecondaryJoystick) {
      //      if (kUseClimber) climber.setDefaultCommand(climberHoldCurrentPosition);
      //      if (kUsePivot) pivot.setDefaultCommand(pivotHoldCurrentPosition);
    }
  }

  private void initCommands() {
    if (kUseDrive)
      slewDrive =
          new SlewDrive(drive, driveJoystick::getTriggerSpeed, driveJoystick::getLeftJoystickX);
    if (!kDiagnostic) {
      if (kUseIntake && kUseFeeder) {
        reverse = new Reverse(intake, feeder);
        load = new Load(intake, feeder);
      }
      if (kUseShooter && kUseFeeder && kUseLights)
        shoot = new Shoot(feeder, shooter, secondaryJoystick::getAButtonIsHeld, lights);
      else if (kUseShooter && kUseFeeder)
        shoot = new Shoot(feeder, shooter, secondaryJoystick::getAButtonIsHeld, null);
    } else {
      // testIntake = new TestIntake(diagnosticIntake, secondaryJoystick);
      if (kUseFeeder) {
        liftIntake = new LiftIntake(feeder, secondaryJoystick);
      }
    }
    if (kUsePivot && kUseClimber) {
      homingSequence = new HomeClimbingSystem(climber, pivot);
    } else if (kUsePivot) homingSequence = new Home(pivot);
    else if (kUseClimber) homingSequence = new Home(climber);

    if (kUsePivot) {
      manualPivot =
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getRightJoystickY);
      pivotHoldCurrentPosition = new HoldCurrentPosition(pivot);
    }
    if (kUseClimber && kUsePivot) {
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      coordinatingClimber = new CoordinatingClimber(climber, pivot, secondaryJoystick);
      highestClimber =
          new SequentialCommandGroup(
              new PositionClimber(climber, HIGHEST.getState()),
              new HoldTargetPosition(climber, HIGHEST.getState().position));
      chinUp =
          new SequentialCommandGroup(
              //                  new PositionClimber(climber, CLEARANCE_HEIGHT.getState()),
              new PositionClimber(climber, CLEARANCE_HEIGHT.getState()),
              new PositionPivot(pivot, MOST_BACK.getState()),
              new PositionClimber(climber, LOWEST.getState()),
              new PositionPivot(pivot, FIRST.getState()),
              new PositionClimber(climber, CLEARANCE_HEIGHT.getState()));
      backbend =
          new SequentialCommandGroup(
              new PositionPivot(pivot, SECOND.getState()),
              new PositionClimber(climber, HIGHEST.getState()));
      backbendAndChinUp =
          new SequentialCommandGroup(
              new ProxyScheduleCommand(chinUp),
              new ProxyScheduleCommand(backbend),
              new ProxyScheduleCommand(coordinatingClimber),
              new ProxyScheduleCommand(chinUp),
              new ProxyScheduleCommand(backbend),
              new ProxyScheduleCommand(coordinatingClimber),
              new ProxyScheduleCommand(chinUp));
      climberHoldCurrentPosition = new HoldCurrentPosition(climber);
    }
    if (kUseFeeder && kUsePivot && kUseIntake && kUseDrive)
      autonomous = new DoubleBallAuto(load, drive, feeder, shooter, lights, navX, vision);

    if (kUseVision) {
      driveWithVision =
          new DriveWithVision(
              drive, driveJoystick::getTriggerSpeed, driveJoystick::getLeftJoystickX, vision);
      if (kUseFeeder && kUseShooter)
        shootWithVision =
            new ShootWithVision(shooter, shoot, vision::hasTarget, vision::getDistance);
    }
  }

  public void configureButtonBindings() {
    if (!kUseController) return;

    if (useDriveJoystick) {
      System.out.println("Setting up drive joystick commands");
      if (kUseDrive) {
        System.out.println("... drive");
        //        buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
        buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
        if (kUseVision) {
          buttonBoard.driveWithVisionButton.whileHeld(driveWithVision);
          if (kUseFeeder && kUseShooter)
            buttonBoard.shootWithVisionButton.whileHeld(shootWithVision);
        }

        if (kUseNavX) {
          System.out.println("... navX");
          //          buttonBoard.autoDriveTestButton.whenPressed(new DriveDistanceProfile(2, 1, 1,
          // drive));
          //          buttonBoard.autoDriveTestReverseButton.whenPressed(
          //              new DriveDistanceProfile(-2, 1, 1, drive));
        }

        if (!kDiagnostic) {
          if (kUseIntake && kUseFeeder) {
            buttonBoard.reverseButton.whileHeld(reverse);
            buttonBoard.liftIntakeButton.whenPressed(new LiftIntake(feeder, secondaryJoystick));
            buttonBoard.loadButton.whileHeld(load);
          }
          if (kUseShooter && kUseFeeder) {
            buttonBoard.shootButton.whileHeld(shoot);
            buttonBoard.toggleShootingHeightButton.whenPressed(
                new InstantCommand(shoot::toggleMode));
          }
        }
      }

      if (useSecondaryJoystick) {
        System.out.println("Setting up secondary joystick commands");
        buttonBoard.homeClimbing.whenPressed(getHomingSequence());
        if (kUsePivot && kUseClimber) {
          buttonBoard.manualClimberButton.whenPressed(
              new ParallelCommandGroup(manualClimber, manualPivot).withName("ManualMode"));
          buttonBoard.coordinatingClimberButton.whenPressed(coordinatingClimber);

          buttonBoard.climberUP.whenPressed(highestClimber);
          buttonBoard.climberDown.whenPressed(chinUp);
          buttonBoard.pivotMostBackButton.whenPressed(backbendAndChinUp);
          buttonBoard.pivotFurthestButton.whenPressed(backbend);
        } else if (kUsePivot) buttonBoard.manualClimberButton.whenPressed(manualPivot);
        else if (kUseClimber) buttonBoard.manualClimberButton.whenPressed(manualClimber);
        if (kUseClimber) {
          System.out.println("... climber and pivot");
          //        buttonBoard.trapezoidClimber.whenPressed(
          //            () -> {
          //
          // climberTrapezoid.updatePosition(climberGoalChooser.getSelected().getState());
          //              climberTrapezoid.schedule();
          //            });
        }
        if (kUsePivot) {
          //        buttonBoard.trapezoidPivot.whenPressed(
          //            () -> {
          //              pivotTrapezoid.updatePosition(pivotGoalChooser.getSelected().getState());
          //              pivotTrapezoid.schedule();
          //            });
        }
      }
    }
  }

  public CommandBase getAutonomous() {
    return autonomous;
  }

  @Config(name = "ON FOR TWO BALLS (pause)", tabName = "Autonomous", defaultValueBoolean = true)
  public void setAutonomous(boolean twoBall) {
    if (twoBall) {
      autonomous = new DoubleBallAuto(load, drive, feeder, shooter, lights, navX, vision);
      System.out.println("Two Ball");
    } else {
      autonomous = new Autonomous(load, shoot, drive, feeder, shooter, lights);
      System.out.println("One ball");
    }
  }

  public void setDrive() {
    if (kUseDrive) {
      drive.setDefaultCommand(slewDrive);
      slewDrive.schedule(true);
    }
  }
}
