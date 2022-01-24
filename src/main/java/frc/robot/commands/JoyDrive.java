// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import utils.drive.Drive;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.TalonDrive;
import utils.joysticks.StormXboxController;

/** An example command that uses an example subsystem. */
public class JoyDrive extends CommandBase {
  private Drive drive;
  private final StormXboxController joystick;
  private DifferentialDrive differentialDrive;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyDrive(Drive subsystem, StormXboxController joy) {
    if (subsystem instanceof SparkDrive) {
      drive = subsystem;
      addRequirements((SparkDrive) drive);
    }

    if (subsystem instanceof TalonDrive) {
      drive = subsystem;
      addRequirements((TalonDrive) drive);
    }
    joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    differentialDrive = drive.getDifferentialDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left, right;
    left = joystick.getLeftJoystickY();
    right = joystick.getRightJoystickX();
    differentialDrive.arcadeDrive(left, -right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
