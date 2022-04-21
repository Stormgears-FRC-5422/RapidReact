// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static DoubleArrayLogEntry shooterDistanceRPSLog;
  private RobotContainer robotContainer;

  public static DoubleArrayLogEntry getShooterDistanceRPSLog() {
    try {
      return shooterDistanceRPSLog;
    } catch (Exception e) {
      e.printStackTrace();
    }
    return null;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    System.out.println(
        "Robot starting at "
            + DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss").format(LocalDateTime.now()));

    // WPIlib logging
    try {
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      shooterDistanceRPSLog = new DoubleArrayLogEntry(log, "/shooter/vision");
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Make Robot Container
    robotContainer = new RobotContainer();

    // Oblog logging
    Logger.configureLoggingAndConfig(robotContainer, false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
  }

  @Override
  public void simulationPeriodic() {
    robotPeriodic();
  }

  @Override
  public void simulationInit() {
    super.simulationInit();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.getAutonomous().schedule();
    robotContainer.feeder.setCoast(); // sets to coast
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //    if (kDiagnostic && kUseFeeder) robotContainer.getLiftIntake().schedule();
    //    CommandScheduler.getInstance().schedule(robotContainer.getHomingSequence());
    CommandScheduler.getInstance().cancelAll();
    robotContainer.setDrive();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.//
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
