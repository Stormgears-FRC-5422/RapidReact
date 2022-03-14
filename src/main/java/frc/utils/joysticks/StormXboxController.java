package frc.utils.joysticks;


import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.Constants.kNullSize;
import static java.lang.Math.abs;

public class StormXboxController extends Joystick implements DriveJoystick {
    private double result; // temp value conveniently and quickly declared one time

    // sticks and triggers
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int zRotation = 4;
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

    @Override
    public double getRightTrigger() {
        return getRawAxis(rightTrigger);
    }

    @Override
    public double getLeftTrigger() {
        return getRawAxis(leftTrigger);
    }

    @Override
    public double getTriggerSpeed(){
        result = getRawAxis(rightTrigger) - getRawAxis(leftTrigger);
        return abs(result) < kNullSize ? 0 : result;
    }

    @Override
    public double getZRotation(){
        result = getRawAxis(zRotation);
        return abs(result) < kNullSize ? 0 : result;
    }

    public boolean getAisPressed() {
        return getRawButtonPressed(AButton);
    }

    public boolean getBisPressed() {
        return getRawButtonPressed(BButton);
    }

    public boolean getXisPressed() {
        return getRawButtonPressed(XButton);
    }

    public boolean getYisPressed() {
        return getRawButtonPressed(YButton);
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


    @Override
    public double getLeftSpeed() {
        result = getRawAxis(leftYAxis);
        return abs(result) < kNullSize ? 0 : result;
    }

    @Override
    public double getRightSpeed() {
        result = getRawAxis(rightYAxis);
        return abs(result) < kNullSize ? 0 : result;
    }

    public double getLeftJoystickX() {
        return getRawAxis(leftXAxis);
    }

    public double getLeftJoystickY() {
        return getRawAxis(leftYAxis);
    }

    public double getRightJoystickX() {
        return getRawAxis(rightXAxis);
    }

    public double getRightJoystickY() {
        return getRawAxis(rightYAxis);
    }
}
