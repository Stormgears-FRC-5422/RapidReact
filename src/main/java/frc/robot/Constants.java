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
  public static final boolean useIntake = StormProp.getBoolean("useIntake", false);
  public static final boolean useShooter = StormProp.getBoolean("useShooter", false);
  public static final boolean useClimber = StormProp.getBoolean("useClimber", false);
  public static final boolean useStatusLights = StormProp.getBoolean("useStatusLights", false);

  // MOTOR CONSTANTS
  public static final String MOTOR_TYPE = StormProp.getString("stormMotorType", "");

  public static final int MASTER_LEFT_ID = StormProp.getInt("masterLeftId", -1);
  public static final int MASTER_RIGHT_ID = StormProp.getInt("masterRightId", -1);
  public static final int SLAVE_LEFT_ID = StormProp.getInt("slaveLeftId", -1);
  public static final int SLAVE_RIGHT_ID = StormProp.getInt("slaveRightId", -1);

  public static final double WHEEL_RADIUS = StormProp.getNumber("wheelRadius", 0d);
  public static final double RATIO = StormProp.getNumber("gearBoxRatio", 0d);

  public static final boolean LEFT_SIDE_INVERTED = StormProp.getBoolean("leftSideInverted", false);
  public static final boolean RIGHT_SIDE_INVERTED =
      StormProp.getBoolean("rightSideInverted", false);
  public static final int SMART_CURRENT_LIMIT = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double NULL_SIZE = StormProp.getNumber("driveNullSize", 0.055);

  public static final double kP = StormProp.getNumber("alignP", 0.05);
  public static final double kI = StormProp.getNumber("alignI", 0.01);
  public static final double kD = StormProp.getNumber("alignD", 0.01);
  public static final double alignTolerance = StormProp.getNumber("toleranceDegrees", 2d);

  public static final double kPDrive = StormProp.getNumber("pDrive", 1.0);
  public static final double kIDrive = StormProp.getNumber("iDrive", 0.0);
  public static final double kDDrive = StormProp.getNumber("dDrive", 0.0);

  public static final int currentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double temperatureRampThreshold =
      StormProp.getInt("SparkMaxTemperatureRampThreshold", 40);
  public static final double temperatureRampLimit =
      StormProp.getInt("SparkMaxTemperatureRampLimit", 55);

  public static final double kMAXVEL = StormProp.getNumber("kMaxVelocity", 0d);
  public static final double kMAXACC = StormProp.getNumber("kMaxAcceleration", 0d);

  private Constants() {
    throw new IllegalStateException("Do not make a Constants class");
  }
}
