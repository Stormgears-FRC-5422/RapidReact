// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.utils.configfile.StormProp;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FRONT_LEFT_ID = StormProp.getInt("frontLeftID", 11);
    public static final int FRONT_RIGHT_ID = StormProp.getInt("frontRightID", 13);
    public static final int REAR_RIGHT_ID = StormProp.getInt("rearRightID", 14);
    public static final int REAR_LEFT_ID = StormProp.getInt("rearLeftID", 12);
    public static final boolean LEFT_SIDE_INVERTED = StormProp.getBoolean("leftSideInverted", false);
    public static final boolean RIGHT_SIDE_INVERTED = StormProp.getBoolean("rightSideInverted", false);
    public static final String MOTOR_TYPE = StormProp.getString("stormMotorType", "Spark");
    public static final double NULL_SIZE = StormProp.getNumber("driveNullSize", 0.055);
  public static final double kP = StormProp.getNumber("alignP", 0.05);
  public static final double kI = StormProp.getNumber("alignI", 0.01);
  public static final double kD = StormProp.getNumber("alignD", 0.01);
  public static final double alignTolerance = StormProp.getNumber("toleranceDegrees", 2d);

    private Constants() {
        throw new IllegalStateException("Do not make a Constants class");
    }   
}
