package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.TestDrive;
import frc.robot.commands.navX.NavXAlign;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.TalonDrive;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

public class RobotContainer {

  private final StormXboxController driveJoystick;
  private final StormXboxController secondaryJoystick;
  private final ButtonBoard buttonBoard;

  private StormDrive drive;
  private NavX navX;

  private TestDrive testDrive;
  private NavXAlign navXAlign;

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
    if (Constants.useDrive) {
      switch (Constants.MOTOR_TYPE) {
        case "Spark":
          drive = new SparkDrive();
          break;
        case "Talon":
          drive = new TalonDrive();
          break;
        default:
      }
    }
    if (Constants.useNavX) navX = new NavX();
  }

  private void initCommands() {
    if (Constants.useDrive) testDrive = new TestDrive(drive, driveJoystick);
    if (Constants.useNavX) navXAlign = new NavXAlign(drive, navX);
  }

  private void configureButtonBindings() {
    if (Constants.useDrive) {
      buttonBoard.autoDriveTestButton.whenPressed(new DriveDistanceProfile(50,3,2,drive));
      buttonBoard.reverseButton.whenPressed(drive::toggleReverse);
      buttonBoard.precisionButton.whenPressed(drive::togglePrecision);
    }
    if (Constants.useNavX) {
      buttonBoard.navXAlignButton.whileHeld(navXAlign);
    }
  }

  private void configureDefaultCommands() {
    if (Constants.useDrive) drive.setDefaultCommand(new TestDrive(drive, driveJoystick));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
