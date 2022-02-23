package frc.robot.commands.navX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.sensors.NavX;
import frc.utils.drive.StormDrive;

import static frc.robot.Constants.*;

public class NavXAlign extends PIDCommand {

  public NavXAlign(StormDrive drive, NavX navX) {
    super(
        new PIDController(alignP, alignI, alignD),
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
