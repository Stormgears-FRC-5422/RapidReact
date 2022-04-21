package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.lights.AddressableLEDBufferRGBW;

import static frc.robot.Constants.*;

public class Lights extends SubsystemBase {
  private final AddressableLED lightStrip = new AddressableLED(kLightsID);
  private final AddressableLEDBufferRGBW buffer = new AddressableLEDBufferRGBW(kLEDLength);
  private final DriverStation.Alliance alliance;
  private final Color8Bit blackColor = new Color8Bit(0, 0, 0);
  private Color8Bit fullColor;
  private Color8Bit halfColor;
  private Color8Bit quarterColor;
  private boolean isShooting;
  private int rainbowHue = 0;
  private int runwayIndex = 0;

  public Lights() {
    lightStrip.setLength(buffer.getEffectiveLength());
    lightStrip.setData(buffer);
    lightStrip.start();

    alliance = DriverStation.getAlliance();
    switch (alliance) {
      case Red:
        fullColor = new Color8Bit(192, 0, 0);
        halfColor = new Color8Bit(96, 0, 0);
        quarterColor = new Color8Bit(48, 0, 0);
        break;
      case Blue:
        fullColor = new Color8Bit(0, 0, 192);
        halfColor = new Color8Bit(0, 0, 96);
        quarterColor = new Color8Bit(0, 0, 48);
        break;
      default:
    }
  }

  @Override
  public void periodic() {
    buffer.useSubstring(kBackLEDStart, kBackLEDLength);
    setAll(fullColor);

    buffer.useSubstring(kRunwayLEDStart, kRunwayLEDLength);
    if (isShooting) {
      runway();
    } else {
      setAll(fullColor);
    }

    buffer.stopSubstring();
    lightStrip.setData(buffer);
  }

  public void setShooting(boolean shooting) {
    this.isShooting = shooting;
  }

  private void setAll(Color8Bit color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  private void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowHue += 3;
    rainbowHue %= 180;
  }

  private void runway() {
    buffer.setLED((runwayIndex) % kRunwayLEDLength, fullColor);
    buffer.setLED((runwayIndex + 1) % kRunwayLEDLength, fullColor);
    buffer.setLED((runwayIndex + 2) % kRunwayLEDLength, fullColor);
    buffer.setLED(((kRunwayLEDLength + runwayIndex - 1) % kRunwayLEDLength), halfColor);
    buffer.setLED(((kRunwayLEDLength + runwayIndex - 2) % kRunwayLEDLength), quarterColor);
    buffer.setLED(((kRunwayLEDLength + runwayIndex - 3) % kRunwayLEDLength), blackColor);

    runwayIndex += 1;
  }
}
