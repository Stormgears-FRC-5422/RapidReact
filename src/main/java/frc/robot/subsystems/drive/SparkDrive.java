// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.utils.drive.StormDrive;
import frc.utils.filters.ExponentialAverage;
import frc.utils.filters.Filter;
import frc.utils.motorcontrol.StormSpark;

import static edu.wpi.first.wpilibj.DriverStation.reportWarning;
import static frc.robot.Constants.*;
import static java.lang.Math.max;

public class SparkDrive extends StormDrive {
  private final StormSpark masterLeft =
      new StormSpark(kMasterLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final StormSpark masterRight =
      new StormSpark(kMasterRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final StormSpark slaveLeft =
      new StormSpark(kSlaveLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final StormSpark slaveRight =
      new StormSpark(kSlaveRightId, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final DifferentialDrive differentialDrive;
  private final double[] conversionFactor = {
    0.0254 * kDriveWheelCircumference / kDriveGearBoxRatio,
    0.0254 * kRightSideSpeedScale * kDriveWheelCircumference / kDriveGearBoxRatio
  }; // set to provide measurement in meters per motor revolution
  private Filter masterRightCurrent;
  private Filter slaveRightCurrent;
  private Filter masterLeftCurrent;
  private Filter slaveLeftCurrent;
  private Filter masterRightTemp;
  private Filter slaveRightTemp;
  private Filter masterLeftTemp;
  private Filter slaveLeftTemp;
  private Filter masterRightVoltage;

  private double delta;
  private int tempWarningCount;

  private SimpleMotorFeedforward m_ff_left, m_ff_right, m_ff_turn;
  private PIDController m_wpi_left_controller;
  private PIDController m_wpi_right_controller;
  private PIDController m_wpi_turn_controller;
  private Filter masterLeftVoltage;

  public SparkDrive() {
    setupMotors();
    setupTempControl();
    setupSlewFactors();
    // Setup PID controllers
    setupControllers();

    differentialDrive = new DifferentialDrive(masterLeft, masterRight);
    differentialDrive.setSafetyEnabled(true);
  }

  protected void setupMotors() {
    System.out.println("Setting up motors");

    for (StormSpark s : getMotors()) {
      s.restoreFactoryDefaults();
      s.setSmartCurrentLimit(kSmartCurrentLimit);
      System.out.println("kDriveIdleModeCoast: " + kDriveIdleModeCoast);
      s.setIdleMode(
          kDriveIdleModeCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    }

    masterLeft.setInverted(kLeftSideInverted);
    slaveLeft.setInverted(kLeftSideInverted);
    masterRight.setInverted(kRightSideInverted);
    slaveRight.setInverted(kRightSideInverted);

    slaveLeft.follow(masterLeft);
    slaveRight.follow(masterRight);

    //  Configure encoders for meters/s
    masterLeft.getEncoder().setPositionConversionFactor(conversionFactor[0]);
    masterRight.getEncoder().setPositionConversionFactor(conversionFactor[1]);
    masterLeft.getEncoder().setVelocityConversionFactor(conversionFactor[0] / 60d);
    masterRight.getEncoder().setVelocityConversionFactor(conversionFactor[1] / 60d);

    // The scale can't be > 1.0 - if that's what we're given, flip the sense by reducing
    // the other side of the drive
    if (kRightSideSpeedScale > 1.0) {
      masterLeft.setSpeedScale(1.0 / kRightSideSpeedScale);
      slaveLeft.setSpeedScale(1.0 / kRightSideSpeedScale);
    } else {
      masterRight.setSpeedScale(kRightSideSpeedScale);
      slaveRight.setSpeedScale(kRightSideSpeedScale);
    }
  }

  protected void setupTempControl() {
    delta =
        max(
            kTemperatureRampLimit - kTemperatureRampThreshold,
            1.0); // Safety margin - don't want divide by 0!

    masterRightCurrent = new ExponentialAverage(masterRight::getOutputCurrent);
    slaveRightCurrent = new ExponentialAverage(slaveRight::getOutputCurrent);
    masterLeftCurrent = new ExponentialAverage(masterLeft::getOutputCurrent);
    slaveLeftCurrent = new ExponentialAverage(slaveLeft::getOutputCurrent);

    masterRightTemp = new ExponentialAverage(masterRight::getMotorTemperature);
    slaveRightTemp = new ExponentialAverage(slaveRight::getMotorTemperature);
    masterLeftTemp = new ExponentialAverage(masterLeft::getMotorTemperature);
    slaveLeftTemp = new ExponentialAverage(slaveLeft::getMotorTemperature);
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  protected StormSpark[] getMotors() {
    return new StormSpark[] {masterLeft, masterRight, slaveLeft, slaveRight};
  }

  @Override
  public void periodic() {
    //    SmartDashboard.putNumber("Drive RightM Current", masterRightCurrent.update());
    //    SmartDashboard.putNumber("Drive RightS Current", slaveRightCurrent.update());
    //    SmartDashboard.putNumber("Drive LeftM Current", masterLeftCurrent.update());
    //    SmartDashboard.putNumber("Drive LeftS Current", slaveLeftCurrent.update());
    //
    //    SmartDashboard.putNumber("Drive RightM Temp", masterRightTemp.update());
    //    SmartDashboard.putNumber("Drive RightS Temp", slaveRightTemp.update());
    //    SmartDashboard.putNumber("Drive LeftM Temp", masterLeftTemp.update());
    //    SmartDashboard.putNumber("Drive LeftS Temp", slaveLeftTemp.update());
    //
    //    SmartDashboard.putNumber("Drive RightM Voltae", masterRightCurrent.update());

    double temp =
        max(
            max(masterRight.getMotorTemperature(), masterLeft.getMotorTemperature()),
            max(slaveRight.getMotorTemperature(), slaveLeft.getMotorTemperature()));

    double multiplier = 1;
    if (temp > kTemperatureRampThreshold) {
      multiplier = max((kTemperatureRampLimit - temp) / delta, 0.0);
    }

    if (tempWarningCount++ % 100 == 0) {
      if (multiplier != 1.0) reportWarning("Safety speed control - factor: " + multiplier, true);
    }

    differentialDrive.setMaxOutput(multiplier);
  }

  protected void setupControllers() {
    // Using WPILib pid controller.  The command must have and manage the trapezoid object.  The
    // drive
    // subsystem doesn't know about it and expects the command to pass position and velocity
    // setpoints
    double[] kV = {kDriveLeftVFF, kDriveRightVFF}; // R2D2 on stand, voltage output
    double[] kP = {kDriveProfileLeftP, kDriveProfileRightP};
    double[] kI = {kDriveProfileLeftI, kDriveProfileRightI};
    double[] kD = {kDriveProfileLeftD, kDriveProfileRightD};

    m_wpi_left_controller = new PIDController(kP[0], kI[0], kD[0]);
    m_wpi_right_controller = new PIDController(kP[1], kI[1], kD[1]);
    m_wpi_turn_controller =
        new PIDController(kDriveTurnProfileP, kDriveTurnProfileI, kDriveTurnProfileD);

    m_ff_left = new SimpleMotorFeedforward(0.3, kV[0]);
    m_ff_right = new SimpleMotorFeedforward(0.3, kV[1]);
    //    SmartDashboard.putNumber("Drive FF Const", kDriveTurnVFF);
    m_ff_turn = new SimpleMotorFeedforward(kDriveTurnSFF, kDriveTurnVFF);
  }

  // Engage the PID controllers to move the robot to the incremental setpoint, must be called every
  // periodic with a new setpoint
  public void setPositionReference(double setPoint) {
    setPositionReferenceWithVelocity(setPoint, 0);
  }

  // Engage the PID controllers to move the robot to the incremental setpoint (use velocity for feed
  // forward)
  // Must be called every periodic with a new setpoint
  public void setPositionReferenceWithVelocity(double setPoint, double velocity) {
    // Get PID output
    double left_out =
        MathUtil.clamp(
            m_wpi_left_controller.calculate(masterLeft.getEncoder().getPosition(), setPoint),
            -kDriveProfileMaxOutput,
            kDriveProfileMaxOutput);
    double right_out =
        MathUtil.clamp(
            m_wpi_right_controller.calculate(masterRight.getEncoder().getPosition(), setPoint),
            -kDriveProfileMaxOutput,
            kDriveProfileMaxOutput);

    //    SmartDashboard.putNumber("Drive Position Left PID output", left_out);
    //    SmartDashboard.putNumber("Drive Position Rifgt PID output", right_out);

    // Get Feedforward
    double ff_left_out = m_ff_left.calculate(velocity);
    double ff_right_out = m_ff_right.calculate(velocity);

    //    SmartDashboard.putNumber("Drive Position Left FF output", ff_left_out);
    //    SmartDashboard.putNumber("Drive Position Rifgt FF output", ff_right_out);

    // Apply to motors
    masterLeft.setVoltage(left_out + ff_left_out);
    masterRight.setVoltage(right_out + ff_right_out);

    //    SmartDashboard.putNumber("Drive Current Distance", getDistance());
    //    SmartDashboard.putNumber("Drive Position Target", setPoint);
  }

  // Engage the PID controllers to move the robot to the incremental setpoint (use velocity for feed
  // forward)
  // Must be called every periodic with a new setpoint.
  // For turns, the measurement must be provided as the drive subsystem does not have access to the
  // gryo
  // subsystem.  The command must manage measurement and setpoint
  public void setTurnPositionReferenceWithVelocity(
      double measurement, double setPoint, double velocity) {
    // Get PID output
    double pid_out =
        MathUtil.clamp(
            m_wpi_turn_controller.calculate(measurement, setPoint),
            -kDriveTurnProfileMaxOutput,
            kDriveTurnProfileMaxOutput);

    //    SmartDashboard.putNumber("Drive Turn Position PID output", pid_out);

    // Get Feedforward
    double ff_out = m_ff_turn.calculate(velocity);

    // Apply to motors
    masterLeft.setVoltage(1 * (pid_out + ff_out));
    masterRight.setVoltage(
        -1
            * (pid_out
                + ff_out)); // Right motor goes backwards for right turn (positive angular velocity)
    //    SmartDashboard.putNumber("Drive Turn feedforward output", ff_out);

    //    SmartDashboard.putNumber("Drive Turn Measurement", measurement);
    //    SmartDashboard.putNumber("Drive Turn Position Target", setPoint);
    //    SmartDashboard.putNumber("Drive Turn Velocity Target", velocity);
  }

  public void resetPosition() {
    masterLeft.getEncoder().setPosition(0);
    masterRight.getEncoder().setPosition(0);
  }

  public double getDistance() {
    return ((masterLeft.getEncoder().getPosition() + masterRight.getEncoder().getPosition()) / 2);
  }

  public double getVelocity() {
    return ((masterLeft.getEncoder().getVelocity() + masterRight.getEncoder().getVelocity()) / 2);
  }
}
