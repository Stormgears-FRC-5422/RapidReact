// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.utils.drive.Drive;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.TalonDrive;
import frc.utils.drive.StormDrive;
import frc.utils.drive.StormMotor;
import frc.utils.joysticks.StormXboxController;

/** An example command that uses an example subsystem. */
public class TestDrive extends CommandBase {
  private StormDrive drive;
  private final StormXboxController joystick;
  private DifferentialDrive differentialDrive;

  public TestDrive(StormDrive stormDrive, StormXboxController joy) {
    drive = stormDrive;
    addRequirements(drive);
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

//    System.out.println("left: " + left + "  right: " + right);
    if(joystick.getAisPressed()){
      slowmode=0.5;
    }
    //differentialDrive.arcadeDrive(slowmode*left, -slowmode*right);
    differentialDrive.tankDrive(left,right,true);

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
