// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.NavXAlign;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SparkDrive;
import frc.robot.subsystems.TalonDrive;
import frc.utils.drive.Drive;
import frc.utils.drive.StormMotor;
import frc.utils.drive.StormMotorType;
import frc.utils.joysticks.StormXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive drive;
  public final StormXboxController stormXboxController;
  private final NavX navX;
  private final NavXAlign navXAlign;
  private final JoyDrive joyDrive;

  public RobotContainer() {
    stormXboxController = new StormXboxController(0);

    if (StormMotor.motorType() == StormMotorType.TALON) drive = new TalonDrive();
    else drive = new SparkDrive();
    joyDrive = new JoyDrive(drive, stormXboxController);
    drive.setDefaultCommand(joyDrive);

    navX = new NavX();
    navXAlign = new NavXAlign(drive, navX);

    configureButtonBindings();

  }


  public Drive getDrive() {
    return drive;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton align = new JoystickButton(stormXboxController, StormXboxController.AButton);
    align.whileHeld(navXAlign);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
