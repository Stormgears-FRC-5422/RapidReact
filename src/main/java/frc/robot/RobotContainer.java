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

  private HomeClimber homeClimber;
  private HomePivot homePivot;
  private ParallelCommandGroup homingSequence;
  private TestClimber testClimber;
  private ClimbingGoal climberGoal = ClimbingGoal.LOW;
  private ClimbingGoal pivotGoal = ClimbingGoal.LOW;

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

  private void initSubsystems() {
    if (kUseDrive)
      switch (kMotorType) {
        case "Spark":
          drive = new SparkDrive();
          break;
        case "Talon":
          drive = new TalonDrive();
          break;
        default:
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
      homePivot = new HomePivot(pivot);
      homeClimber = new HomeClimber(climber);
      homingSequence = new ParallelCommandGroup(homeClimber, homePivot);
    }
  }

  private void configureButtonBindings() {
    if (!kUseController) return;

    if (kUseDrive) {
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
      if (kUseNavX)
        buttonBoard.autoDriveTestButton.whenPressed(new DriveDistanceProfile(2, 1, 1, drive));
    }

    if (kUseNavX) buttonBoard.navXAlignButton.whileHeld(navXAlign);

    if (!kDiagnostic) {
      if (kUseIntake && kUseFeeder) buttonBoard.loadButton.whileHeld(load);
      if (kUseShooter && kUseFeeder) {
        buttonBoard.shootButton.whileHeld(shoot);
        buttonBoard.toggleShootingHeightButton.whenPressed(new InstantCommand(shoot::toggleMode));
      }
    } // else {
    //      if (kUseIntake)
    // buttonBoard.selectIntakeButton.whenPressed(diagnosticIntake::setModeIntake);
    //      if (kUseFeeder)
    // buttonBoard.selectFeederButton.whenPressed(diagnosticIntake::setModeFeeder);
    //      if (kUseShooter)
    // buttonBoard.selectShooterButton.whenPressed(diagnosticIntake::setModeShooter);
    //    }
    if (kUseNavX) buttonBoard.navXAlignButton.whileHeld(navXAlign);

    if (kUseClimber && kUsePivot){
      buttonBoard.trapezoidClimber.whenPressed(
          () -> {
            climberGoal = climberGoal.toggle();
            new PositionClimber(climber, climberGoal.state).schedule();
          });
      buttonBoard.trapezoidPivot.whenPressed(
          () -> {
            pivotGoal = pivotGoal.toggle();
            new PositionPivot(pivot, pivotGoal.state).schedule();
          });
      buttonBoard.manualClimberButton.whenPressed(testClimber);
      //      buttonBoard.trapezoidClimber.whenPressed(positionClimber);
      //      buttonBoard.trapezoidPivot.whenPressed(positionPivot);
      //      buttonBoard.homeClimberButton.whenPressed(homeClimber);
      //      buttonBoard.homePivotButton.whenPressed(homePivot);
    }
  }

  private void configureDefaultCommands() {
    if (kUseDrive) drive.setDefaultCommand(new SlewDrive(drive, driveJoystick));
    if (kUseClimber && kUsePivot) {
      //      climber.setDefaultCommand(holdClimber);
      //      pivot.setDefaultCommand(holdPivot);
    }
//    if (kDiagnostic) {diagnosticIntake.setDefaultCommand(testIntake);

    // See robot.teleopInit() for climber scheduling. It cannot be a default command

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
    LOW(25),
    HIGH(195);
    public final TrapezoidProfile.State state;

    ClimbingGoal(double state) {
      this.state = new TrapezoidProfile.State(state, 0);
    }

    ClimbingGoal toggle() {
      return this == LOW ? HIGH : LOW;
    }
  }
}
