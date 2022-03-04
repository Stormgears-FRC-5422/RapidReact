// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SparkDrive;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

/** An example command that uses an example subsystem. */
public class TestDrive extends CommandBase {
  private final SparkDrive drive;
  private final StormXboxController joystick;
  private DifferentialDrive differentialDrive;

  public TestDrive(SparkDrive sparkDrive, StormXboxController joy) {
    this.drive = sparkDrive;
    addRequirements(sparkDrive);
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
    double slowmode=1;
    left = joystick.getLeftJoystickY();
    right = joystick.getRightJoystickY();

    if(joystick.getAisPressed()){
      slowmode=0.5;
    }
    differentialDrive.tankDrive(left, right, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
