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
    public static final boolean useDrive = StormProp.getBoolean("useDrive", false);
    public static final boolean useNavX = StormProp.getBoolean("useNavX", false);
    public static final boolean diagnostic = StormProp.getBoolean("useDiagnostic", false);
    public static final boolean useIntake = StormProp.getBoolean("useIntake", false);
    public static final boolean useFeeder = StormProp.getBoolean("useFeeder", false);
    public static final boolean useShooter = StormProp.getBoolean("useShooter", false);
    public static final boolean useClimber = StormProp.getBoolean("useClimber", false);
    public static final boolean useStatusLights = StormProp.getBoolean("useStatusLights", false);

    // MOTOR CONSTANTS
    public static final String MOTOR_TYPE = StormProp.getString("stormMotorType", "");

    public static final int MASTER_LEFT_ID = StormProp.getInt("masterLeftId", -1);
    public static final int MASTER_RIGHT_ID = StormProp.getInt("masterRightId", -1);
    public static final int SLAVE_LEFT_ID = StormProp.getInt("slaveLeftId", -1);
    public static final int SLAVE_RIGHT_ID = StormProp.getInt("slaveRightId", -1);

    public static final boolean LEFT_SIDE_INVERTED = StormProp.getBoolean("leftSideInverted", false);
    public static final boolean RIGHT_SIDE_INVERTED = StormProp.getBoolean("rightSideInverted", false);
    public static final int SMART_CURRENT_LIMIT = StormProp.getInt("SparkMaxCurrentLimit", 1);
    public static final double NULL_SIZE = StormProp.getNumber("driveNullSize", 0.055);

    public static final int currentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
    public static final double temperatureRampThreshold = StormProp.getInt("SparkMaxTemperatureRampThreshold", 40);
    public static final double temperatureRampLimit = StormProp.getInt("SparkMaxTemperatureRampLimit", 55);

    public static final int SHOOTER_ID = StormProp.getInt("shooterId", -1);
    public static final int INTAKE_ID = StormProp.getInt("intakeId", -1);
    public static final int FEEDER_ID = StormProp.getInt("feederId", -1);
    public static final double shooterLowRPM = StormProp.getNumber("shooterLowRPM", 0d);

    private Constants() {
        throw new IllegalStateException("Do not make a Constants class");
    }   
}
