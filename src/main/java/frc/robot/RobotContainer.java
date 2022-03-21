package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.ballHandler.LiftIntake;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.ballHandler.TestIntake;
import frc.robot.commands.climber.ManualClimber;
import frc.robot.commands.climber.hold.HoldCurrentPosition;
import frc.robot.commands.climber.home.Home;
import frc.robot.commands.climber.home.HomeClimbingSystem;
import frc.robot.commands.climber.trapezoid.ClimbingGoal;
import frc.robot.commands.climber.trapezoid.PivotGoal;
import frc.robot.commands.climber.trapezoid.PositionClimber;
import frc.robot.commands.climber.trapezoid.PositionPivot;
import frc.robot.commands.drive.SlewDrive;
import frc.robot.subsystems.ballHandler.DiagnosticIntake;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.drive.TalonDrive;
import frc.robot.subsystems.sensors.NavX;
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
public class RobotContainer {

  private final boolean useDriveJoystick;
  private final boolean useSecondaryJoystick;
  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;
  /** Declare subsystems - initialize below */
  private StormDrive drive;

  @Log private NavX navX;
  private DiagnosticIntake diagnosticIntake;
  @Log private Shooter shooter;
  private Feeder feeder;
  private Intake intake;
  private TestIntake testIntake;
  private LiftIntake liftIntake;

  @Config.Command(tabName = "Driver", name = "Up Climber")
  private final SequentialCommandGroup goIn;

  @Config.Command(tabName = "Driver", name = "UP Climber TRAVERSE")
  private final SequentialCommandGroup highestClimber;

  @Config.Command(tabName = "Driver", name = "Down Climber")
  private final SequentialCommandGroup lowestClimber;

  @Config.Command(tabName = "Driver", name = "Chin Up pivot")
  private final SequentialCommandGroup firstpivot;

  @Config.Command(tabName = "Driver", name = "Level 2 pivot")
  private final SequentialCommandGroup secondPivot;

  @Config.Command(tabName = "Driver", name = "back Pivot")
  private final SequentialCommandGroup mostBack;

  private Load load;
  @Log private Shoot shoot;
  private CommandBase homingSequence;
  private ManualClimber manualClimber;
  private ManualClimber manualPivot;
  @Log.Exclude private HoldCurrentPosition climberHoldCurrentPosition;
  @Log.Exclude private HoldCurrentPosition pivotHoldCurrentPosition;
  @Log private PositionClimber climberTrapezoid;
  @Log private PositionPivot pivotTrapezoid;
  @Log private Climber climber;
  @Log private Pivot pivot;
  private Autonomous autonomous;
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
    goIn =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> climberTrapezoid.updatePosition(ClimbingGoal.FIRST.getState())),
            new ScheduleCommand(climberTrapezoid));
    highestClimber =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> climberTrapezoid.updatePosition(ClimbingGoal.HIGHEST.getState())),
            new ScheduleCommand(climberTrapezoid));
    lowestClimber =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> climberTrapezoid.updatePosition(ClimbingGoal.LOWEST.getState())),
            new ScheduleCommand(climberTrapezoid));
    firstpivot =
        new SequentialCommandGroup(
            new InstantCommand(() -> pivotTrapezoid.updatePosition(PivotGoal.FIRST.getState())),
            new ProxyScheduleCommand(pivotTrapezoid));
    secondPivot =
        new SequentialCommandGroup(
            new InstantCommand(() -> pivotTrapezoid.updatePosition(PivotGoal.SECOND.getState())),
            new ProxyScheduleCommand(pivotTrapezoid));
    mostBack =
        new SequentialCommandGroup(
            new InstantCommand(() -> pivotTrapezoid.updatePosition(PivotGoal.MOST_BACK.getState())),
            new ProxyScheduleCommand(pivotTrapezoid));
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
    }
  }

  public StormDrive getDrive() {
    return drive;
  }

  public CommandBase getHomingSequence() {
    return homingSequence == null ? new InstantCommand() : homingSequence;
  }

  public void configureDefaultCommands() {
    if (useDriveJoystick) {
    }
    if (useSecondaryJoystick) {
      if (kUseClimber) climber.setDefaultCommand(climberHoldCurrentPosition);
      if (kUsePivot) pivot.setDefaultCommand(pivotHoldCurrentPosition);
    }
  }

  private void initCommands() {
    if (!kDiagnostic) {
      if (kUseIntake && kUseFeeder) load = new Load(intake, feeder);
      if (kUseShooter && kUseFeeder) shoot = new Shoot(feeder, shooter);
    } else {
      // testIntake = new TestIntake(diagnosticIntake, secondaryJoystick);
      if (kUseFeeder) {
        liftIntake = new LiftIntake(feeder, secondaryJoystick);
      }
    }
    if (kUsePivot && kUseClimber) homingSequence = new HomeClimbingSystem(climber, pivot);
    else if (kUsePivot) homingSequence = new Home(pivot);
    else if (kUseClimber) homingSequence = new Home(climber);

    if (kUsePivot) {
      manualPivot =
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getRightJoystickY);
      pivotHoldCurrentPosition = new HoldCurrentPosition(pivot);
      pivotTrapezoid = new PositionPivot(pivot, () -> ClimbingGoal.LOWEST.getState());
    }
    if (kUseClimber) {
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      climberHoldCurrentPosition = new HoldCurrentPosition(climber);
      climberTrapezoid = new PositionClimber(climber, () -> ClimbingGoal.HIGHEST.getState());
    }
    if (kUseFeeder && kUsePivot && kUseIntake && kUseDrive)
      autonomous = new Autonomous(load, shoot, drive);
  }

  public void configureButtonBindings() {
    if (!kUseController) return;

    if (useDriveJoystick) {
      System.out.println("Setting up drive joystick commands");
      if (kUseDrive) {
        System.out.println("... drive");
        //        buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
        buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
        if (kUseNavX) {
          System.out.println("... navX");
          //          buttonBoard.autoDriveTestButton.whenPressed(new DriveDistanceProfile(2, 1, 1,
          // drive));
          //          buttonBoard.autoDriveTestReverseButton.whenPressed(
          //              new DriveDistanceProfile(-2, 1, 1, drive));
        }

      if (!kDiagnostic) {
        if (kUseIntake && kUseFeeder) buttonBoard.loadButton.whileHeld(load);
        if (kUseShooter && kUseFeeder) {
          buttonBoard.shootButton.whileHeld(shoot);
          buttonBoard.toggleShootingHeightButton.whenPressed(new InstantCommand(shoot::toggleMode));
        }
      }

    }

    if (useSecondaryJoystick) {
      System.out.println("Setting up secondary joystick commands");
      buttonBoard.homeClimbing.whenPressed(getHomingSequence());
      if (kUsePivot && kUseClimber) {
        buttonBoard.manualClimberButton.whenPressed(
            new ParallelCommandGroup(manualClimber, manualPivot).withName("ManualMode"));
          buttonBoard.climberUP.whenPressed(highestClimber);
          buttonBoard.climberDown.whenPressed(lowestClimber);
          buttonBoard.pivotIN.whenPressed(firstpivot);
          buttonBoard.pivotOut.whenPressed(mostBack);
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

  public Autonomous getAutonomous() {
    return autonomous;
  }

  public void setDrive() {
    if (kUseDrive) {
      slewDrive = new SlewDrive(drive, driveJoystick);
      drive.setDefaultCommand(slewDrive);
      slewDrive.schedule(false);
    }
  }

  //  @Config.ToggleSwitch(
  //      name = "Climber Limits",
  //      defaultValue = true,
  //      tabName = "Driver")
  //  private boolean toggleLimits(boolean toggle) {
  //    if (toggle && climber == null) climber.enableAllLimits();
  //    if (toggle && climber == null) climber.disableAllLimits();
  //    return climber.isAllLimitsEnabled();
  //  }
  //
  //  @Config.ToggleSwitch(
  //      name = "Pivot Limits",
  //      defaultValue = true,
  //      tabName = "Driver")
  //  private boolean toggleLimitsPivot(boolean toggle) {
  //    if (toggle && pivot != null) pivot.enableAllLimits();
  //    if (toggle && pivot != null) pivot.disableAllLimits();
  //    assert pivot != null;
  //    return pivot.isAllLimitsEnabled();
  //  }
  //
  //  @Log.CameraStream(tabName = "Driver")
  //  VideoSource cameraStream = new Stream()

}
