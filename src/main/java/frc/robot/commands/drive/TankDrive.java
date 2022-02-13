package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SafeDrive;
import frc.utils.joysticks.DriveJoystick;

public class TankDrive extends CommandBase {
  private final SafeDrive drive;
  private final DriveJoystick joystick;

  public TankDrive(SafeDrive drive, DriveJoystick joystick) {
    addRequirements(drive);

    this.drive = drive;
    this.joystick = joystick;
  }

  @Override
  public void execute() {
    drive.tankDrive(-joystick.getLeftSpeed(), -joystick.getRightSpeed());
  }
}
