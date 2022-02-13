package frc.utils.motorcontrol;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class StormTalon extends WPI_TalonSRX {

  public StormTalon(int deviceID) {
    super(deviceID);
  }

  @Override
  public void set(double speed) {
    // Could put safety features - e.g. tempeature control - here.
    super.set(speed);
  }
}
