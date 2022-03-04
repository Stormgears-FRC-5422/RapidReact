package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NavX extends SubsystemBase {
  private final AHRS ahrs;

  //PID controller for rotation
  private PIDController rotatePID = new PIDController(Constants.kPRotate, Constants.kIRotate, Constants.kDRotate);

  public NavX() {
    ahrs = new AHRS();
    ahrs.enableLogging(true);
  }

  public double getAngle() {
    return Math.toRadians(getAngleDegrees());
  }

  public double getAngleDegrees() {
    return ahrs.getYaw();
  }

  public double calculateRotateVel(double degrees) {
    return rotatePID.calculate(getAngleDegrees(), degrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle radians", getAngle());
    SmartDashboard.putNumber("angle degrees", getAngleDegrees());
  }

  public void resetAngle() {
    ahrs.reset();
  }
}
