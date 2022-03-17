package frc.robot.subsystems.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NavX extends SubsystemBase {
  private final AHRS ahrs;

  public NavX() {
    ahrs = new AHRS();
    ahrs.enableLogging(true);
  }

  public double getAngle() {
    return Math.toRadians(getAngleDegrees());
  }

  public double getTotalAngleDegrees() {
    return(Constants.kNavXGyroScaleFactor * ahrs.getAngle());
  }
  
  public double getAngleDegrees() {
    return(ahrs.getYaw() * Constants.kNavXGyroScaleFactor);
  }

  public void reset(){
    ahrs.zeroYaw();
  }

  public double getHeading(){
    return ahrs.getYaw();
    //return -26d;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle radians", getAngle());
    SmartDashboard.putNumber("angle degrees", getAngleDegrees());
    }
}
