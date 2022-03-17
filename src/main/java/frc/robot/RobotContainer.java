package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballHandler.LiftIntake;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.ballHandler.TestIntake;
import frc.robot.commands.climber.*;
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

  private ParallelCommandGroup homingSequence;
  private TestClimber testClimber;
  private ClimbingGoal climberGoal = ClimbingGoal.LOW;
  private PivotGoal pivotGoal = PivotGoal.LOW;

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  private boolean useDriveJoystick;
  private boolean useSecondaryJoystick;

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

    if (kUseClimber && kUsePivot) {
      testClimber = new TestClimber(climber,pivot,secondaryJoystick);
      homingSequence = new ParallelCommandGroup(new Home(pivot), new Home(climber));
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
      if (kUseClimber && kUsePivot) {
        System.out.println("... climber and pivot");
        buttonBoard.manualClimberButton.whenPressed(testClimber);
//        buttonBoard.homeClimbing.whenPressed(getHomingSequence());
//
//        buttonBoard.trapezoidClimber.whenPressed(
//                () -> {
//                  climberGoal = climberGoal.toggle();
//                  new PositionClimber(climber, climberGoal.state)
//                          .andThen(new Hold(climber, climberGoal.state.position))
//                          .schedule();
//                });
//        buttonBoard.trapezoidPivot.whenPressed(
//                () -> {
//                  pivotGoal = pivotGoal.toggle();
//                  new PositionPivot(pivot, pivotGoal.state)
//                          .andThen(new Hold(pivot, pivotGoal.state.position))
//                          .schedule();
//                });
      }
    }

  }

  private void configureDefaultCommands() {
    if (useDriveJoystick) {
      if (kUseDrive) drive.setDefaultCommand(new SlewDrive(drive, driveJoystick));
      //if (kUseClimber && kUsePivot) {
      //      climber.setDefaultCommand(new Hold(climber));
      //      pivot.setDefaultCommand(new Hold(pivot));
      //}
    }
  }

  public StormDrive getDrive() {
    return drive;
  }

  public LiftIntake getLiftIntake() {
    return liftIntake;
  }

  public TestClimber getTestClimber() {
    return testClimber;
  }

  public ParallelCommandGroup getHomingSequence() {
    return homingSequence;
  }

  enum ClimbingGoal {
    LOW(10),
    HIGH(275);
    public TrapezoidProfile.State state;

    ClimbingGoal(double state) {
      this.state = new TrapezoidProfile.State(state, 0);
    }

    ClimbingGoal toggle() {
      return this == LOW ? HIGH : LOW;
    }
  }

  enum PivotGoal {
    LOW(15),
    HIGH(52);
    public TrapezoidProfile.State state;

    PivotGoal(double state) {
      this.state = new TrapezoidProfile.State(state, 0);
    }

    PivotGoal toggle() {
      return this == LOW ? HIGH : LOW;
    }
  }
}
