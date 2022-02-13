package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.NavX;
import frc.utils.drive.Drive;

import static frc.robot.Constants.*;

public class NavXAlign extends PIDCommand {

  public NavXAlign(Drive drive, NavX navX) {
    super(
        new PIDController(alignkP, alignkI, alignkD),
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
