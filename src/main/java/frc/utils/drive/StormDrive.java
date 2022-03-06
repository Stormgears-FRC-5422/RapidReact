package frc.utils.drive;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.kSlewRate;
import static frc.robot.Constants.kTurnSlewRate;
import static java.lang.Math.abs;

public abstract class StormDrive extends SubsystemBase {
  protected boolean reverse = false;
  protected boolean precision = false;
  protected double slewRateValue;
  protected double turnSlewRateValue;


  public abstract DifferentialDrive getDifferentialDrive();

  public void simulationInit() {  }

  public void simulationPeriodic() {  }

  public void rotate(double zRotation) {
    if (abs(zRotation) > 1) {
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

  public boolean getPrecision() {
    return this.precision;
  }

  public void setPrecision(boolean p) {
    this.precision = p;
  }

  public void togglePrecision() {
    precision = !precision;
  }

  protected abstract MotorController[] getMotors();

  public double getSlewRate() {
    return(slewRateValue);
  }

  public double getTurnSlewRate() {
    return(turnSlewRateValue);
  }

  protected void setupSlewFactors() {
    slewRateValue = kSlewRate;
    turnSlewRateValue = kTurnSlewRate;

    NetworkTableEntry shuffleSlewRate = Shuffleboard.getTab("Control")
            .add("Drive Slew Rate", slewRateValue)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max",  2 * slewRateValue))
            .getEntry();

    NetworkTableEntry mShuffleTurnSlewRate = Shuffleboard.getTab("Control")
            .add("Turn Slew Rate", turnSlewRateValue)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 2 * turnSlewRateValue))
            .getEntry();

    shuffleSlewRate.addListener(event -> slewRateValue = event.value.getDouble(),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    mShuffleTurnSlewRate.addListener(event -> turnSlewRateValue = event.value.getDouble(),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

}
