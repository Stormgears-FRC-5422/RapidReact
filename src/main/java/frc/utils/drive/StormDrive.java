package frc.utils.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class StormDrive extends SubsystemBase {
  protected boolean reverse = false;
  protected boolean precision = false;

  private double pDrive = Constants.kPDrive;
  private double iDrive = Constants.kIDrive;
  private double dDrive = Constants.kDDrive;
  //PID controller for drive
  protected PIDController drivePID = new PIDController(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive);
  public abstract double calculateDriveVel(double goalRadians);
  //PID controller for rotation
  protected PIDController rotatePID = new PIDController(Constants.kPRotate, Constants.kIRotate, Constants.kDRotate);
  public abstract double calculateRotateVel(double goalRadians);

  public abstract void ResetEncoders();
  public abstract double getDistance();

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

  protected abstract MotorController[] getMotors();
}
