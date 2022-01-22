// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StormMotorType;

/** An example command that uses an example subsystem. */
public class JoyDrive extends CommandBase {
  private final Drive drive;
  private StormMotorType motorType;
  private DifferentialDrive differentialDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyDrive(Drive subsystem) {
    drive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorType = drive.motorType;
    differentialDrive = drive.getDifferentialDrive(motorType);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differentialDrive.arcadeDrive(0.5, 0);
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
