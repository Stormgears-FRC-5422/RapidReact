// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import utils.drive.StormMotorType;
import utils.joysticks.DriveJoystick;
import utils.joysticks.StormXboxController;

/** An example command that uses an example subsystem. */
public class JoyDrive extends CommandBase {
  private final Drive drive;
  private StormXboxController joystick;
  private DifferentialDrive differentialDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyDrive(Drive subsystem, StormXboxController joy) {
    drive = subsystem;
    joystick = joy;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StormMotorType motorType = drive.getMotorType();
    differentialDrive = drive.getDifferentialDrive(motorType);
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
