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
import frc.robot.commands.climber.trapezoid.*;
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
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
  @Config.Command(tabName = "Driver", name = "UP Climber TRAVERSE")
  private SequentialCommandGroup highestClimber;
  @Config.Command(tabName = "Driver", name = "Down Climber")
  private SequentialCommandGroup lowestClimber;
  @Config.Command(tabName = "Driver", name = "Chin Up pivot")
  private SequentialCommandGroup firstpivot;
  @Config.Command(tabName = "Driver", name = "Level 2 pivot")
  private SequentialCommandGroup secondPivot;

  @Config.Command(tabName = "Driver", name = "back Pivot")
  private SequentialCommandGroup furthest;
  /** Declare subsystems - initialize below */
  private StormDrive drive;

  @Log private NavX navX;
  private DiagnosticIntake diagnosticIntake;
  @Log private Shooter shooter;
  private Feeder feeder;
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
  @Log private PositionClimber climberTrapezoid;
  @Log private PositionPivot pivotTrapezoid;

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
    if (!kDiagnostic) {
      if (kUseIntake && kUseFeeder) {
        reverse = new Reverse(intake, feeder);
        load = new Load(intake, feeder);
      }
      if (kUseShooter && kUseFeeder) shoot = new Shoot(feeder, shooter);
    } else {
      // testIntake = new TestIntake(diagnosticIntake, secondaryJoystick);
      if (kUseFeeder) {
        liftIntake = new LiftIntake(feeder, secondaryJoystick);
      }
    }
    if (kUsePivot && kUseClimber) {
      homingSequence = new HomeClimbingSystem(climber, pivot);
      highestClimber =
              new SequentialCommandGroup(
                      new PositionClimber(climber, ClimbingGoal.HIGHEST.getState()),
                      new HoldTargetPosition(climber, ClimbingGoal.HIGHEST.getState().position));
      lowestClimber =
              new SequentialCommandGroup(
                      new ParallelCommandGroup(
                              new PositionClimber(climber, new State(65, 0)),
                              new PositionPivot(pivot, PivotGoal.MOST_BACK.getState())),
                      new PositionClimber(climber, ClimbingGoal.LOWEST.getState()),
                      new ParallelRaceGroup(
                              new HoldTargetPosition(climber, ClimbingGoal.LOWEST.getState().position),
                              new PositionPivot(pivot, PivotGoal.FIRST.getState()))
                              .withName("Hold Climb"),
                      new ParallelRaceGroup(
                              new HoldTargetPosition(pivot, PivotGoal.FIRST.getState().position),
                              new PositionClimber(climber, new State(65, 0)))
                              .withName("Hold Pivot"));
      firstpivot =
              new SequentialCommandGroup(
                      new PositionPivot(pivot, PivotGoal.FIRST.getState()),
                      new HoldTargetPosition(pivot, PivotGoal.FIRST.getState().position));
      secondPivot =
              new SequentialCommandGroup(
                      new PositionPivot(pivot, PivotGoal.SECOND.getState()),
                      new HoldTargetPosition(pivot, PivotGoal.SECOND.getState().position));
      furthest =
              new SequentialCommandGroup(
                      new PositionPivot(pivot, PivotGoal.FURTHEST.getState()),
                      new HoldTargetPosition(pivot, PivotGoal.FURTHEST.getState().position));
    } else if (kUsePivot) homingSequence = new Home(pivot);
    else if (kUseClimber) homingSequence = new Home(climber);

    if (kUsePivot) {
      manualPivot =
          new ManualClimber(pivot, secondaryJoystick, secondaryJoystick::getRightJoystickY);
      pivotHoldCurrentPosition = new HoldCurrentPosition(pivot);
    }
    if (kUseClimber) {
      manualClimber =
          new ManualClimber(climber, secondaryJoystick, secondaryJoystick::getLeftJoystickY);
      coordinatingClimber =
              new CoordinatingClimber(climber,pivot,secondaryJoystick);

      climberHoldCurrentPosition = new HoldCurrentPosition(climber);
    }
    if (kUseFeeder && kUsePivot && kUseIntake && kUseDrive)
      autonomous = new DoubleBallAuto(load, shoot, drive, navX);
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
          buttonBoard.climberDown.whenPressed(lowestClimber);
          buttonBoard.pivotIN.whenPressed(firstpivot);
          buttonBoard.pivotOut.whenPressed(furthest);
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
      autonomous = new DoubleBallAuto(load, shoot, drive, navX);
      System.out.println("Two Ball");
    } else {
      autonomous = new Autonomous(load, shoot, drive);
      System.out.println("One ball");
    }
  }

  public void setDrive() {
    if (kUseDrive) {
      slewDrive = new SlewDrive(drive, driveJoystick);
      drive.setDefaultCommand(slewDrive);
      slewDrive.schedule(false);
    }
  }

}
