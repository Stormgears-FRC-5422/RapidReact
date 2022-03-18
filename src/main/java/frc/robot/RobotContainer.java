package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballHandler.LiftIntake;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.ballHandler.TestIntake;
import frc.robot.commands.climber.ManualClimber;
import frc.robot.commands.climber.hold.HoldCurrentPosition;
import frc.robot.commands.climber.home.Home;
import frc.robot.commands.climber.home.HomeClimbingSystem;
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
  private TestIntake testIntake;
  private LiftIntake liftIntake;

  private Climber climber;
  private Pivot pivot;

  private Load load;
  private Shoot shoot;
  private NavXAlign navXAlign;

  private CommandBase homingSequence;
  private final boolean useDriveJoystick;
  private final boolean useSecondaryJoystick;
  private ManualClimber manualClimber;
  private ManualClimber manualPivot;
  private SendableChooser<ClimbingGoal> climberGoalChooser;
  private SendableChooser<PivotGoal> pivotGoalChooser;

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;
  private HoldCurrentPosition climberHoldCurrentPosition;
  private HoldCurrentPosition pivotHoldCurrentPosition;

  public RobotContainer() {
    driveJoystick = new StormXboxController(0);
    secondaryJoystick = new StormXboxController(1);
    buttonBoard = ButtonBoard.getInstance(driveJoystick, secondaryJoystick);

    useDriveJoystick = (kUseController && kUseJoystick0 && driveJoystick.isConnected());
    useSecondaryJoystick = (kUseController && kUseJoystick1 && secondaryJoystick.isConnected());
    System.out.println("useDriveJoystick is " + useDriveJoystick + ", useSecondaryJoystick is " + useSecondaryJoystick);

    initSubsystems();
    initCommands();
    configureButtonBindings();
    configureDefaultCommands();
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
      //diagnosticIntake = new DiagnosticIntake();
      if (kUseFeeder) feeder = new Feeder();
    }
    else {
      if (kUseShooter) shooter = new Shooter();
      if (kUseFeeder) feeder = new Feeder();
      if (kUseIntake) intake = new Intake();
    }
  }

  private void initCommands() {
    if (!kDiagnostic) {
      if (kUseIntake && kUseFeeder) load = new Load(intake, feeder);
      if (kUseShooter && kUseFeeder) shoot = new Shoot(feeder, shooter);
    } else {
      //testIntake = new TestIntake(diagnosticIntake, secondaryJoystick);
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
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      pivotHoldCurrentPosition = new HoldCurrentPosition(pivot);
      initPivotChooser();
    }
    if (kUseClimber) {
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      climberHoldCurrentPosition = new HoldCurrentPosition(climber);
      initClimberChooser();
    }
  }

  private void configureButtonBindings() {
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
          buttonBoard.autoDriveTestReverseButton.whenPressed(new DriveDistanceProfile(-2, 1, 1, drive));
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
        buttonBoard.trapezoidClimber.whenPressed(
            new PositionClimber(climber, climberGoalChooser.getSelected().state));
      }
      if (kUsePivot) {
        buttonBoard.trapezoidPivot.whenPressed(
            new PositionPivot(pivot, pivotGoalChooser.getSelected().state));
      }
    }

  }

  private void configureDefaultCommands() {
    if (useDriveJoystick) {
      if (kUseDrive) drive.setDefaultCommand(new SlewDrive(drive, driveJoystick));
      if (kUseClimber) climber.setDefaultCommand(climberHoldCurrentPosition);
      if (kUsePivot) pivot.setDefaultCommand(pivotHoldCurrentPosition);
    }
  }

  public StormDrive getDrive() {
    return drive;
  }

  public CommandBase getHomingSequence() {
    return homingSequence == null ? new InstantCommand() : homingSequence;
  }

  private void initPivotChooser() {
    pivotGoalChooser = new SendableChooser<>();
    for (PivotGoal goal : PivotGoal.values()) pivotGoalChooser.addOption(goal.name(), goal);
    Shuffleboard.getTab("Pivot").add("Pivot Goal", pivotGoalChooser);
  }

  private void initClimberChooser() {
    climberGoalChooser = new SendableChooser<>();
    for (ClimbingGoal goal : ClimbingGoal.values()) climberGoalChooser.addOption(goal.name(), goal);
    Shuffleboard.getTab("Climber").add("Climbing Goal", climberGoalChooser);
  }

  private void shuffleBoardDriverTab() {
    // TODO add Camera Feed
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.add("PivotChooser", pivotGoalChooser);
    driverTab.add(
        "Move Pivot To Goal", new PositionPivot(pivot, pivotGoalChooser.getSelected().state));
    driverTab.add("ClimberChooser", climberGoalChooser);
    driverTab.add(
        "Move Climber To Goal",
        new PositionClimber(climber, climberGoalChooser.getSelected().state));
    driverTab.add(
        "Climber To Value",
        builder ->
            builder.addDoubleProperty(
                "Climber Value",
                () -> 0,
                value -> new PositionClimber(climber, new TrapezoidProfile.State(value, 0))));
    driverTab.add(
        "Pivot To Value",
        builder ->
            builder.addDoubleProperty(
                "Pivot Value",
                () -> 0,
                value -> new PositionPivot(pivot, new TrapezoidProfile.State(value, 0))));
    driverTab.add("Climber Position", climber);
    driverTab.add("Pivot Position", pivot);
  }

  enum PivotGoal {
    MOST_BACK(5),
    FIRST(52),
    //    SECOND(150),
    FURTHEST(145);

    TrapezoidProfile.State state;

    PivotGoal(double state) {
      this.state = new TrapezoidProfile.State(state, 0);
    }

  }

  enum ClimbingGoal {
    LOWEST(10),
    SECOND(80),
    FIRST(275),
    HIGHEST(275);

    TrapezoidProfile.State state;

    ClimbingGoal(double state) {
      this.state = new TrapezoidProfile.State(state, 0);
    }

  }

}