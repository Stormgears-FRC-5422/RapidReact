package frc.robot.subsystems.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class NavX extends SubsystemBase implements Loggable {
  private final AHRS ahrs;

  public NavX() {
    ahrs = new AHRS();
    ahrs.enableLogging(true);
  }

  @Log(name = "Angle Radians")
  public double getAngle() {
    return Math.toRadians(getAngleDegrees());
  }

  @Log(name = "Cumulative Angle Degrees (Scaled)")
  public double getTotalAngleDegrees() {
    return (Constants.kNavXGyroScaleFactor * ahrs.getAngle());
  }

  @Log(name = "Heading Degrees (Scaled)")
  public double getAngleDegrees() {
    return (ahrs.getYaw() * Constants.kNavXGyroScaleFactor);
  }

  //    @Override
  //    public void periodic() {
  //      SmartDashboard.putNumber("angle radians", getAngle());
  //      SmartDashboard.putNumber("angle degrees", getAngleDegrees());
  //      }
}
