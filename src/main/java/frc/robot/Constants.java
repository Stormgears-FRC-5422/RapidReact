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
  public static final boolean kUseController = StormProp.getBoolean("useController", false);
  public static final boolean kUseDrive = StormProp.getBoolean("useDrive", false);
  public static final boolean kUseNavX = StormProp.getBoolean("useNavX", false);
  public static final boolean kDiagnostic = StormProp.getBoolean("useDiagnostic", false);
  public static final boolean kUseIntake = StormProp.getBoolean("useIntake", false);
  public static final boolean kUseFeeder = StormProp.getBoolean("useFeeder", false);
  public static final boolean kUseShooter = StormProp.getBoolean("useShooter", false);
  public static final boolean kUseClimber = StormProp.getBoolean("useClimber", false);
  public static final boolean kUsePivot = StormProp.getBoolean("usePivot", false);
  public static final boolean kUseStatusLights = StormProp.getBoolean("useStatusLights", false);

  // MOTOR CONSTANTS
  public static final String kMotorType = StormProp.getString("stormMotorType", "");
  public static final double kSlewRate = StormProp.getNumber("slewRate", 0.0);
  public static final double kTurnSlewRate = StormProp.getNumber("turnSlewRate", 0.0);


  public static final int kMasterLeftId = StormProp.getInt("masterLeftId", -1);
  public static final int kMasterRightId = StormProp.getInt("masterRightId", -1);
  public static final int kSlaveLeftId = StormProp.getInt("slaveLeftId", -1);
  public static final int kSlaveRightId = StormProp.getInt("slaveRightId", -1);

  public static final boolean kLeftSideInverted = StormProp.getBoolean("leftSideInverted", false);
  public static final boolean kRightSideInverted = StormProp.getBoolean("rightSideInverted", false);
  public static final double kRightSideScale = StormProp.getNumber("rightSideScale", 1.0);

  public static final int kSmartCurrentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double kNullSize = StormProp.getNumber("driveNullSize", 0.055);
  public static final double kXPrecision = StormProp.getNumber("xPrecision", 0.0);
  public static final double kZPrecision = StormProp.getNumber("zPrecision", 0.0);

  public static final double kAlignP = StormProp.getNumber("alignP", 0.05);
  public static final double kAlignI = StormProp.getNumber("alignI", 0.01);
  public static final double kAlignD = StormProp.getNumber("alignD", 0.01);
  public static final double kAlignTolerance = StormProp.getNumber("toleranceDegrees", 2.0);

  public static final int kCurrentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double kTemperatureRampThreshold = StormProp.getInt("SparkMaxTemperatureRampThreshold", 40);
  public static final double kTemperatureRampLimit = StormProp.getInt("SparkMaxTemperatureRampLimit", 55);

  public static final int kShooterId = StormProp.getInt("shooterId", -1);
  public static final int kIntakeId = StormProp.getInt("intakeId", -1);
  public static final int kFeederId = StormProp.getInt("feederId", -1);

  public static final double kFeederSpeed = StormProp.getNumber("feederSpeed", 0.0);
  public static final double kIntakeSpeed = StormProp.getNumber("intakeSpeed", 0.0);

  public static final double kShooterP = StormProp.getNumber("shooterP", 0.0);
  public static final double kShooterI = StormProp.getNumber("shooterI", 0.0);
  public static final double kShooterD = StormProp.getNumber("shooterD", 0.0);
  public static final double kShooterF = StormProp.getNumber("shooterF", 0.0);
  public static final double kShooterLowRPS = StormProp.getNumber("shooterLowRPS", 0.0);
  public static final double kShooterHighRPS = StormProp.getNumber("shooterHighRPS", 0.0);

  public static final int kClimberLeftId = StormProp.getInt("climberLeftId", -1);
  public static final int kClimberRightId = StormProp.getInt("climberRightId", -1);
  public static final int kPivotLeftId = StormProp.getInt("pivotLeftId", -1);
  public static final int kPivotRightId = StormProp.getInt("pivotRightId", -1);

  public static final boolean kClimberLeftInverted = StormProp.getBoolean("climberLeftInverted", false);
  public static final boolean kClimberRightInverted = StormProp.getBoolean("climberRightInverted", false);
  public static final boolean kPivotLeftInverted = StormProp.getBoolean("pivotLeftInverted", false);
  public static final boolean kPivotRightInverted = StormProp.getBoolean("pivotRightInverted", false);

  public static final double kClimberSpeed = StormProp.getNumber("climberSpeed", 0.0);
  public static final double kPivotSpeed = StormProp.getNumber("pivotSpeed", 0.0);

  private Constants() {
    throw new IllegalStateException("Do not make a Constants class");
  }
}
