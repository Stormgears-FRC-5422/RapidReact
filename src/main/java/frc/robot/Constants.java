// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import utils.StormProp;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int frontLeftID = StormProp.getInt("frontLeftID", -1);
    public static final int frontRightID = StormProp.getInt("frontRightID", -1);
    public static final int rearRightID = StormProp.getInt("rearRightID", -1);
    public static final int rearLeftID = StormProp.getInt("rearLeftID", -1);


    public static final boolean leftSideInverted = StormProp.getBoolean("leftSideInverted", false);
    public static final boolean rightSideInverted = StormProp.getBoolean("rightSideInverted", false);

    public static final String motorType = StormProp.getString("stormMotorType", "Spark");
}
