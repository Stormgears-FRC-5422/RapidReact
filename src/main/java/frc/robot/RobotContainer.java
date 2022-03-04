package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.TestDrive;
import frc.robot.commands.drive.TrapezoidalPIDDrive;
import frc.robot.commands.drive.TrapezoidalPIDRotate;
import frc.robot.commands.navX.NavXAlign;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.TalonDrive;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

public class RobotContainer {

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  private SparkDrive drive;
  private NavX navX;

  private TestDrive testDrive;
  private NavXAlign navXAlign;
  private TrapezoidalPIDRotate trapRotate;

  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(45, 0);

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
    if (Constants.useNavX) navX = new NavX();
    if (Constants.useDrive) {
      drive = new SparkDrive();
    }
  }

  private void initCommands() {
    if (Constants.useDrive) testDrive = new TestDrive(drive, driveJoystick);
    if (Constants.useNavX) navXAlign = new NavXAlign(drive, navX);

    //Init trapezoidal rotate command
    trapRotate = new TrapezoidalPIDRotate(drive, navX, goalState);
  }

  private void configureButtonBindings() {
    if (Constants.useDrive) {
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
    }
    if (Constants.useNavX) {
      buttonBoard.navXAlignButton.whenPressed(trapRotate);
    }
  }

  private void configureDefaultCommands() {
    if (Constants.useDrive) drive.setDefaultCommand(new TestDrive(drive, driveJoystick));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
