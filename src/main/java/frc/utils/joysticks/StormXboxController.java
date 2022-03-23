package frc.utils.joysticks;


import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.Constants.kStickNullSize;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class StormXboxController extends Joystick implements DriveJoystick {

    // sticks and triggers
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int rightXAxis = 4;
    public static final int rightYAxis = 5;

    // buttons
    public static final int AButton = 1;
    public static final int BButton = 2;
    public static final int XButton = 3;
    public static final int YButton = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;
    public static final int littleLeftButton = 7;
    public static final int littleRightButton = 8;
    public static final int stickLeftButton = 9;
    public static final int stickRightButton = 10;

    public StormXboxController(int port) {
        super(port);
    }

    protected double applyNullZone(double value) {
        if (abs(value) < kStickNullSize) return 0;

        // Scale up from 0 rather than jumping to the input value
        return ( (value - signum(value) * kStickNullSize) / (1.0 - kStickNullSize) );
    }

    @Override
    public double getRightTrigger() {
        return getRawAxis(rightTrigger);
    }

    @Override
    public double getLeftTrigger() {
        return getRawAxis(leftTrigger);
    }

    @Override
    public double getTriggerSpeed(){ return getRawAxis(rightTrigger) - getRawAxis(leftTrigger); }

  public boolean getAButton() {
    return getRawButton(AButton);
  }

    public boolean getBisPressed() {
        return getRawButtonPressed(BButton);
    }

  public boolean getXButton() {
    return getRawButton(XButton);
    }

  public boolean getYButton() {
    return getRawButton(YButton);
    }

    public boolean getBackIsPressed() {
        return getRawButtonPressed(littleLeftButton);
    }
    
    public boolean getRightBumperIsPressed() {return getRawButtonPressed(rightBumper); }

    public boolean getLeftBumperIsPressed() {
        return getRawButtonPressed(leftBumper);
    }

    public boolean getRotateIsPressed() {
        return getRawButtonPressed(littleRightButton);
    }

    public boolean getAlignedIsPressed(){
        return getRawButtonPressed(stickLeftButton);
    }

    public boolean getRightBumperIsHeld() {return getRawButton(rightBumper); }

    public boolean getLeftBumperIsHeld() {
        return getRawButton(leftBumper);
    }

  public boolean getRightLittleButtonIsHeld() {
    return getRawButton(littleRightButton);
  }

  public boolean getLeftLittleButtonIsHeld() {
    return getRawButton(littleRightButton);
  }

    public double getLeftJoystickX() {
        return applyNullZone(getRawAxis(leftXAxis));
    }

    public double getLeftJoystickY() { return applyNullZone(getRawAxis(leftYAxis)); }

    public double getRightJoystickX() {
        return applyNullZone(getRawAxis(rightXAxis));
    }

    public double getRightJoystickY() {
        return applyNullZone(getRawAxis(rightYAxis));
    }
}
