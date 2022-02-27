package frc.utils.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StormDrive extends SubsystemBase {
  protected boolean reverse = false;
  protected boolean precision = false;

  public abstract DifferentialDrive getDifferentialDrive();

  public void rotate(double zRotation) {
    if (Math.abs(zRotation) > 1) {
      System.out.println("Not valid " + zRotation);
      zRotation = 1;
    }
    getDifferentialDrive().arcadeDrive(0, zRotation);
  }

  public boolean getReverse() {
    return this.reverse;
  }

  public void setReverse(boolean r) {
    this.reverse = r;
  }

  public void toggleReverse() {
    reverse = !reverse;
    System.out.println("reverse = " + reverse);
  }

  public boolean getPrecisions() {
    return this.precision;
  }

  public void setPrecision(boolean p) {
    this.precision = p;
  }

  public void togglePrecision() {
    precision = !precision;
    System.out.println("precision = " + precision);
  }

  // Set the PID reference
  public void setPositionReference(double setPoint) {};
  // Reset the encoder position
  public void resetPosition() {};
  // Set the acceleration profile
  public void setMaxAccel(double velocity) {};
  // Set the velocity profile
  public void setMaxVelocity(double velocity) {};
  
  // provide method to access encoder distance
  public double getDistance() {
    return(0);
  }
  // provide method to access encoder velocity
  public double getVelocity() {
    return(0);
  }

  protected abstract MotorController[] getMotors();
}
