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
  public static final boolean kUseJoystick0 = StormProp.getBoolean("useJoystick0", false);
  public static final boolean kUseJoystick1 = StormProp.getBoolean("useJoystick1", false);

  public static final boolean kUseDrive = StormProp.getBoolean("useDrive", false);
  public static final boolean kUseNavX = StormProp.getBoolean("useNavX", false);
  public static final boolean kDiagnostic = StormProp.getBoolean("useDiagnostic", false);
  public static final boolean kUseIntake = StormProp.getBoolean("useIntake", false);
  public static final boolean kUseFeeder = StormProp.getBoolean("useFeeder", false);
  public static final boolean kUseShooter = StormProp.getBoolean("useShooter", false);
  public static final boolean kUseClimber = StormProp.getBoolean("useClimber", false);
  public static final boolean kUsePivot = StormProp.getBoolean("usePivot", false);
  public static final boolean kUseStatusLights = StormProp.getBoolean("useStatusLights", false);

  public static final boolean kSquareDriveInputs = StormProp.getBoolean("squareDriveInputs", true);
  public static final String kDriveStyle = StormProp.getString("driveStyle", "arcade");
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
  public static final double kRightSideSpeedScale = StormProp.getNumber("rightSideSpeedScale", 1.0);

  public static final boolean kDriveIdleModeCoast =
      StormProp.getBoolean("driveIdleModeCoast", true);

  public static final int kSmartCurrentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double kStickNullSize = StormProp.getNumber("stickNullSize", 0.1);
  public static final double kXPrecision = .65; // StormProp.getNumber("xPrecision", 0.0);
  public static final double kZPrecision = .45; // StormProp.getNumber("zPrecision", 0.0);

  public static final double kAlignP = StormProp.getNumber("alignP", 0.05);
  public static final double kAlignI = StormProp.getNumber("alignI", 0.01);
  public static final double kAlignD = StormProp.getNumber("alignD", 0.01);
  public static final double kAlignTolerance = StormProp.getNumber("toleranceDegrees", 2.0);

  public static final int kCurrentLimit = StormProp.getInt("SparkMaxCurrentLimit", 1);
  public static final double kTemperatureRampThreshold =
      StormProp.getInt("SparkMaxTemperatureRampThreshold", 40);
  public static final double kTemperatureRampLimit =
      StormProp.getInt("SparkMaxTemperatureRampLimit", 55);

  public static final int kShooterId = StormProp.getInt("shooterId", -1);
  public static final int kIntakeId = StormProp.getInt("intakeId", -1);
  public static final int kFeederId = StormProp.getInt("feederId", -1);

  public static final double kClimberHomeCurrentLimit =
      StormProp.getNumber("climberHomeCurrentLimit", 0.0);
  public static final double kClimberHomeSetSpeed = StormProp.getNumber("climberHomeSetSpeed", 0.0);
  public static final double kPivotHomeCurrentLimit =
      StormProp.getNumber("pivotHomeCurrentLimit", 0.0);
  public static final double kPivotHomeSetSpeed = StormProp.getNumber("pivotHomeSetSpeed", 0.0);



  public static final double kFeederSpeed = StormProp.getNumber("feederSpeed", 0.0);
  public static final double kIntakeSpeed = StormProp.getNumber("intakeSpeed", 0.0);

  public static final double kShooterP = StormProp.getNumber("shooterP", 0.0);
  public static final double kShooterI = StormProp.getNumber("shooterI", 0.0);
  public static final double kShooterD = StormProp.getNumber("shooterD", 0.0);
  public static final double kShooterS = StormProp.getNumber("shooterS", 0.0);
  public static final double kShooterV = StormProp.getNumber("shooterV", 0.0);
  public static final double kShooterA = StormProp.getNumber("shooterA", 0.0);
  public static final double kShooterLowRPS = StormProp.getNumber("shooterLowRPS", 0.0);
  public static final double kShooterHighRPS = StormProp.getNumber("shooterHighRPS", 0.0);
  public static final double kShooterTolerance = StormProp.getNumber("shootTolerance", 0.0);
  public static final double kShooterkITolerance =
      StormProp.getNumber("shooterIntegralTolerance", 0.0);

  public static final int kClimberLeftId = StormProp.getInt("climberLeftId", -1);
  public static final int kClimberRightId = StormProp.getInt("climberRightId", -1);
  public static final int kPivotLeftId = StormProp.getInt("pivotLeftId", -1);
  public static final int kPivotRightId = StormProp.getInt("pivotRightId", -1);

  public static final boolean kClimberLeftInverted =
      StormProp.getBoolean("climberLeftInverted", false);
  public static final boolean kClimberRightInverted =
      StormProp.getBoolean("climberRightInverted", false);
  public static final boolean kPivotLeftInverted = StormProp.getBoolean("pivotLeftInverted", false);
  public static final boolean kPivotRightInverted =
      StormProp.getBoolean("pivotRightInverted", false);

  public static final double kClimberSpeed = StormProp.getNumber("climberSpeed", 0.0);
  public static final double kPivotSpeed = StormProp.getNumber("pivotSpeed", 0.0);

  public static final double kClimberMaxVelocity = StormProp.getNumber("climberMaxVelocity", 0.0);
  public static final double kClimberMaxAcceleration =
      StormProp.getNumber("climberMaxAcceleration", 0.0);
  public static final double kPivotMaxVelocity = StormProp.getNumber("pivotMaxVelocity", 0.0);
  public static final double kPivotMaxAcceleration =
      StormProp.getNumber("pivotMaxAcceleration", 0.0);

  public static final double kLeftClimberP = StormProp.getNumber("leftClimberP", 0.0);
  public static final double kLeftClimberI = StormProp.getNumber("leftClimberI", 0.0);
  public static final double kLeftClimberD = StormProp.getNumber("leftClimberD", 0.0);

  public static final double kRightClimberP = StormProp.getNumber("rightClimberP", 0.0);
  public static final double kRightClimberI = StormProp.getNumber("rightClimberI", 0.0);
  public static final double kRightClimberD = StormProp.getNumber("rightClimberD", 0.0);

  public static final double kLeftPivotP = StormProp.getNumber("leftPivotP", 0.0);
  public static final double kLeftPivotI = StormProp.getNumber("leftPivotI", 0.0);
  public static final double kLeftPivotD = StormProp.getNumber("leftPivotD", 0.0);

  public static final double kRightPivotP = StormProp.getNumber("rightPivotP", 0.0);
  public static final double kRightPivotI = StormProp.getNumber("rightPivotI", 0.0);
  public static final double kRightPivotD = StormProp.getNumber("rightPivotD", 0.0);

  public static final double kCLimberForwardLimit =
      StormProp.getNumber("climberSoftLimitForward", 0.0);
  public static final double kCLimberReverseLimit =
      StormProp.getNumber("climberSoftLimitReverse", 0.0);
  public static final double kPivotForwardLimit = StormProp.getNumber("pivotSoftLimitForward", 0.0);
  public static final double kPivotReverseLimit = StormProp.getNumber("pivotSoftLimitReverse", 0.0);

  public static final double kDriveProfileLeftP = StormProp.getNumber("driveProfileLeftP", 0.0);
  public static final double kDriveProfileLeftI = StormProp.getNumber("driveProfileLeftI", 0.0);
  public static final double kDriveProfileLeftD = StormProp.getNumber("driveProfileLeftD", 0.0);
  public static final double kDriveLeftVFF = StormProp.getNumber("driveLeftVFF", 0.0);

  public static final double kDriveProfileRightP = StormProp.getNumber("driveProfileRightP", 0.0);
  public static final double kDriveProfileRightI = StormProp.getNumber("driveProfileRightI", 0.0);
  public static final double kDriveProfileRightD = StormProp.getNumber("driveProfileRightD", 0.0);
  public static final double kDriveRightVFF = StormProp.getNumber("driveRightVFF", 0.0);

  public static final double kDriveTurnProfileP = StormProp.getNumber("driveTurnProfileP", 0.0);
  public static final double kDriveTurnProfileI = StormProp.getNumber("driveTurnProfileI", 0.0);
  public static final double kDriveTurnProfileD = StormProp.getNumber("driveTurnProfileD", 0.0);
  public static final double kDriveTurnVFF = StormProp.getNumber("driveTurnVFF", 0.0);
  public static final double kDriveTurnSFF =
      StormProp.getNumber("driveTurnSFF", 0.0); // minmum input for movement

  public static final double kDriveProfileMaxOutput =
      StormProp.getNumber("driveProfileMaxOutput", 0.0);
  public static final double kDriveTurnProfileMaxOutput =
      StormProp.getNumber("driveTurnProfileMaxOutput", 0.0);

  public static final double kDriveGearBoxRatio = StormProp.getNumber("driveGearBoxRatio", 0.0);
  public static final double kDriveWheelCircumference =
      StormProp.getNumber("driveWheelCircumference", 0.0);

  public static final double kNavXGyroScaleFactor = StormProp.getNumber("navXGyroScaleFactor", 1.0);
  public static final double kClimberMidpoint = StormProp.getNumber("climberMidpoint", 0.0);

  public static final double kClimberCushion = StormProp.getNumber("climberCushion", 0.0);
  public static final double kClimberCushionFloor = StormProp.getNumber("climberCushionFloor", 0.0);
  public static final double kPivotCushion = StormProp.getNumber("pivotCushion", 0.0);
  public static final double kPivotCushionFloor = StormProp.getNumber("pivotCushionFloor", 0.0);

  public static final double kClimberBaseLength = StormProp.getNumber("climberBaseLength", 0.0);
  public static final double kClimberLengthPerRotation = StormProp.getNumber("climberLengthPerRotation", 0.0);
  public static final double kPivotLength = StormProp.getNumber("pivotLength", 0.0);
  public static final double kPivotLengthPerRotation = StormProp.getNumber("pivotLengthPerRotation", 0.0);
  public static final double kBarDistance = StormProp.getNumber("barDistance", 0.0);
  public static final double kPivotSupportLength = StormProp.getNumber("pivotSupportLength", 0.0);
  public static final double kPivotMountLength= StormProp.getNumber("pivotMountLength", 0.0);
  public static final double kPivotLeadOffset = StormProp.getNumber("pivotLeadOffset", 0.0);
  
  private Constants() {
    throw new IllegalStateException("Do not make a Constants class");
  }
}
