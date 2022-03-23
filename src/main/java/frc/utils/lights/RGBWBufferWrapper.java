package frc.utils.lights;

// Based on the AddressableLEDBuffer code from wpilib. Unfortunately I couldn't just subclass it, and they didn't make an interface...
// The good news is that the protocol is really easy - the main part is just sending an array of bytes, so we can fool the
// RGB variant by sending the right bytes along.
// To use, the example code does something like this in robotInit();
//    Specify PWM port - Must be a PWM header, not MXP or DIO
//    m_led = new AddressableLED(0);
//
//    // Reuse buffer
//    // Length is expensive to set, so only set it once, then just update data
//    m_ledBuffer = new RGBWBufferWrapper(16);
//
//    // Call the effectiveLength to get the correct number of bytes set up. As opposed to length() which is the number of RGBW devices
//    m_led.setLength(m_ledBuffer.getEffectiveLength());
//
//    // Set the data - you can do this in periodic, too.
//    m_led.setData(m_ledBuffer.getLEDBuffer());
//    // Only need to start onces
//    m_led.start();

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RGBWBufferWrapper {
    AddressableLEDBuffer ledBuffer;
    int ledLength;


    public RGBWBufferWrapper(int length) {
        ledLength = length;
        ledBuffer = new AddressableLEDBuffer( (length * 4 + 2) / 3 );
    }


    public void setRGBW(int index, int r, int g, int b, int w) {
        // actual array order for the NEOPixels we have is GRBW
        // So, where are the bytes for index i? They might straddle two pixels...
        // array locations = (i-1) * 4 + 0,1,2,3 for G R B W
        // these need to be mapped to (j-1) * 3  + 0, 1, 2 for G R B
        // This is further confused because G comes before R in the array, but not in the function call.

        // byte  0    1    2 |  3    4    5 |  6    7    8 |  9   10   11
        // RGB   0    0    0 |  1    1    1 |  2    2    2 |  3    3    3
        //       G    R    B |  G    R    B |  G    R    B |  G    R    B
        // RGBW  0    0    0    0 |  1    1    1    1 |  2    2    2    2
        //       G    R    B    W |  G    R    B    W |  G    R    B    W
        //      %0    g    r    b |  w  c1.r c1.b|
        //      %1                |c0.g   g    r |  b    w  c1.b|
        //      %2                                c0.g c0.r   g |  r    b    w

        Color8Bit c0;
        Color8Bit c1;

        int aG = index * 4;
        int j = aG / 3;

        // Stash the current color, if any. We need to merge our new colors into these slots
        c0 = ledBuffer.getLED8Bit(j);
        c1 = ledBuffer.getLED8Bit(j+1);

        // which byte are we aligned with in the RGB world? Set the new colors, but keep the other bytes as-is
        switch (aG % 3) {
            case 0:
                ledBuffer.setRGB( j, r, g, b);
                ledBuffer.setRGB(j+1, c1.red, w, c1.blue);
                break;
            case 1:
                ledBuffer.setRGB( j, g, c0.green, r);
                ledBuffer.setRGB(j+1, w, b, c1.blue);
                break;
            case 2:
                ledBuffer.setRGB( j, c0.red, c0.green, g);
                ledBuffer.setRGB(j+1, b, r, w);
                break;
        }

    }


    public void setRGB(int index, int r, int g, int b) {
        setRGBW(index, r,g,b,0);
    }


    public void setLED(int index, Color8Bit color) {
        setRGB(index, color.red, color.green, color.blue);
    }


    public void setLED(int index, Color color) {
        setRGB(index, (int) (color.red * 127), (int) (color.green * 127), (int) (color.blue * 127));
    }

    /**
     * Sets a specific led in the buffer.
     *
     * @param index the index to write
     * @param h the h value [0-180]
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    @SuppressWarnings("ParameterName")
    public void setHSV(final int index, final int h, final int s, final int v) {
        if (s == 0) {
            setRGB(index, v, v, v);
            return;
        }

        final int region = h / 30;
        final int remainder = (h - (region * 30)) * 6;

        final int p = (v * (255 - s)) >> 8;
        final int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        final int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

        switch (region) {
            case 0:
                setRGB(index, v, t, p);
                break;
            case 1:
                setRGB(index, q, v, p);
                break;
            case 2:
                setRGB(index, p, v, t);
                break;
            case 3:
                setRGB(index, p, q, v);
                break;
            case 4:
                setRGB(index, t, p, v);
                break;
            default:
                setRGB(index, v, p, q);
                break;
        }
    }


    public int getLength() {
        return ledLength;
    }


    public int getEffectiveLength() {
        return ledBuffer.getLength();
    }


    public AddressableLEDBuffer getLEDBuffer() {
        return ledBuffer;
    }

}
