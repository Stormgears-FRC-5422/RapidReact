package frc.robot.commands.navX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SparkDrive;
import frc.utils.drive.StormDrive;

import static frc.robot.Constants.*;

public class NavXAlign extends PIDCommand {

  public NavXAlign(SparkDrive drive, NavX navX) {
    super(
        new PIDController(kP, kI, kD),
        navX::getAngle,
        () -> SmartDashboard.getNumber("targetAngle", Math.toRadians(180)),
        drive::rotate,
        drive);
    }

  @Override
  public void initialize() {
    super.initialize();
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Math.toRadians(alignTolerance));
    SmartDashboard.putData("align PID", this.m_controller);
  }
}
