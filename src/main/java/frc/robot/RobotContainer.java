package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.commands.drive.SlewDrive;
import frc.robot.commands.navX.NavXAlign;
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
  private final InstantCommand goFirst =
      new InstantCommand(
          () -> {
            climberTrapezoid.updatePosition(ClimbingGoal.FIRST.getState());
            climberTrapezoid.schedule();
          });

  @Config.Command(tabName = "Driver", name = "UP Climber TRAVERSE")
  private final InstantCommand yert =
      new InstantCommand(
          () -> {
            climberTrapezoid.updatePosition(ClimbingGoal.HIGHEST.getState());
            climberTrapezoid.schedule();
          });

  private Load load;
  @Log private Shoot shoot;
  private NavXAlign navXAlign;
  private CommandBase homingSequence;
  private ManualClimber manualClimber;
  private ManualClimber manualPivot;
  private SendableChooser<ClimbingGoal> climberGoalChooser;
  private SendableChooser<PivotGoal> pivotGoalChooser;
  @Log.Exclude private HoldCurrentPosition climberHoldCurrentPosition;
  @Log.Exclude private HoldCurrentPosition pivotHoldCurrentPosition;
  @Log private PositionClimber climberTrapezoid;

  @Config.Command(tabName = "Driver", name = "Chin Up pivot")
  private final InstantCommand goIn =
      new InstantCommand(
          () -> {
            pivotTrapezoid.updatePosition(PivotGoal.FIRST.getState());
            pivotTrapezoid.schedule();
          });

  @Config.Command(tabName = "Driver", name = "Down Climber")
  private final InstantCommand goLow =
      new InstantCommand(
          () -> {
            climberTrapezoid.updatePosition(ClimbingGoal.LOWEST.getState());
            climberTrapezoid.schedule();
          });

  @Config.Command(tabName = "Driver", name = "Level 2 pivot")
  private final InstantCommand itDonotMatter =
      new InstantCommand(
          () -> {
            pivotTrapezoid.updatePosition(PivotGoal.SECOND.getState());
            pivotTrapezoid.schedule();
          });

  @Log private PositionPivot pivotTrapezoid;

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

  @Config.Command(tabName = "Driver", name = "back Pivot")
  private final InstantCommand backPivot =
      new InstantCommand(
          () -> {
            pivotTrapezoid.updatePosition(PivotGoal.MOST_BACK.getState());
            pivotTrapezoid.schedule();
          });

  @Log private Climber climber;
  @Log private Pivot pivot;
  private Autonomous autonomous;

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

  public StormDrive getDrive() {
    return drive;
  }

  public CommandBase getHomingSequence() {
    return homingSequence == null ? new InstantCommand() : homingSequence;
  }

  public void configureDefaultCommands() {
    if (useDriveJoystick) {
      if (kUseDrive) drive.setDefaultCommand(new SlewDrive(drive, driveJoystick));
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

    if (kUseNavX) navXAlign = new NavXAlign(drive, navX);

    if (kUsePivot && kUseClimber) homingSequence = new HomeClimbingSystem(climber, pivot);
    else if (kUsePivot) homingSequence = new Home(pivot);
    else if (kUseClimber) homingSequence = new Home(climber);

    if (kUsePivot) {
      manualPivot =
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getRightJoystickY);
      pivotHoldCurrentPosition = new HoldCurrentPosition(pivot);
      initPivotChooser();
      pivotTrapezoid = new PositionPivot(pivot, pivotGoalChooser.getSelected()::getState);
    }
    if (kUseClimber) {
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      climberHoldCurrentPosition = new HoldCurrentPosition(climber);
      initClimberChooser();
      climberTrapezoid = new PositionClimber(climber, climberGoalChooser.getSelected()::getState);
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
        buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
        buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
        if (kUseNavX) {
          System.out.println("... navX");
          buttonBoard.navXAlignButton.whileHeld(navXAlign);
          buttonBoard.autoDriveTestButton.whenPressed(new DriveDistanceProfile(2, 1, 1, drive));
          buttonBoard.autoDriveTestReverseButton.whenPressed(
              new DriveDistanceProfile(-2, 1, 1, drive));
        }
      }

      if (!kDiagnostic) {
        if (kUseIntake && kUseFeeder) buttonBoard.loadButton.whileHeld(load);
        if (kUseShooter && kUseFeeder) {
          buttonBoard.shootButton.whileHeld(shoot);
          buttonBoard.toggleShootingHeightButton.whenPressed(new InstantCommand(shoot::toggleMode));
        }
      }

      if (kUseNavX) buttonBoard.navXAlignButton.whileHeld(navXAlign);
    }

    if (useSecondaryJoystick) {
      System.out.println("Setting up secondary joystick commands");
      buttonBoard.homeClimbing.whenPressed(getHomingSequence());
      if (kUsePivot && kUseClimber) {
        buttonBoard.manualClimberButton.whenPressed(
            new ParallelCommandGroup(manualClimber, manualPivot).withName("ManualMode"));
      } else if (kUsePivot) buttonBoard.manualClimberButton.whenPressed(manualPivot);
      else if (kUseClimber) buttonBoard.manualClimberButton.whenPressed(manualClimber);
      if (kUseClimber) {
        System.out.println("... climber and pivot");
        climberTrapezoid = new PositionClimber(climber, climberGoalChooser.getSelected()::getState);
        buttonBoard.trapezoidClimber.whenPressed(
            () -> {
              climberTrapezoid.updatePosition(climberGoalChooser.getSelected().getState());
              climberTrapezoid.schedule();
            });
      }
      if (kUsePivot) {
        pivotTrapezoid = new PositionPivot(pivot, pivotGoalChooser.getSelected()::getState);
        buttonBoard.trapezoidPivot.whenPressed(
            () -> {
              pivotTrapezoid.updatePosition(pivotGoalChooser.getSelected().getState());
              pivotTrapezoid.schedule();
            });
      }
    }
  }

  private void initPivotChooser() {
    pivotGoalChooser = new SendableChooser<>();
    for (PivotGoal goal : PivotGoal.values()) pivotGoalChooser.addOption(goal.name(), goal);
    pivotGoalChooser.setDefaultOption(PivotGoal.values()[0].name(), PivotGoal.values()[0]);
    //        Shuffleboard.getTab("Driver").add("Pivot Goal", pivotGoalChooser);
  }

  private void initClimberChooser() {
    climberGoalChooser = new SendableChooser<>();
    for (ClimbingGoal goal : ClimbingGoal.values()) climberGoalChooser.addOption(goal.name(), goal);
    climberGoalChooser.setDefaultOption(ClimbingGoal.values()[0].name(), ClimbingGoal.values()[0]);
    //        Shuffleboard.getTab("Driver").add("Climbing Goal", climberGoalChooser);
  }

  public Autonomous getAutonomous() {
    return autonomous;
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
