package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.lights.AddressableLEDBufferRGBW;

import static frc.robot.Constants.kLEDLength;
import static frc.robot.Constants.kLightsID;

public class Lights extends SubsystemBase {
  private final AddressableLED lightStrip = new AddressableLED(kLightsID);
  private final AddressableLEDBufferRGBW buffer = new AddressableLEDBufferRGBW(kLEDLength);

  private int rainbowHue = 0;

  public Lights() {
    lightStrip.setLength(buffer.getEffectiveLength());
    lightStrip.setData(buffer);
    lightStrip.start();
  }

  @Override
  public void periodic() {
    rainbow();
    lightStrip.setData(buffer);
  }

  private void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowHue += 3;
    rainbowHue %= 180;
  }
}
