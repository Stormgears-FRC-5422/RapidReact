// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.drive.StormDrive;
import frc.utils.filters.ExponentialAverage;
import frc.utils.filters.Filter;
import frc.utils.motorcontrol.StormSpark;

import static edu.wpi.first.wpilibj.DriverStation.reportWarning;
import static frc.robot.Constants.*;
import static java.lang.Math.max;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;

public class SparkDrive extends StormDrive {
    private final StormSpark masterLeft = new StormSpark(kMasterLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(kMasterRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(kSlaveLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(kSlaveRightId, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DifferentialDrive differentialDrive;

    private Filter masterRightCurrent;
    private Filter slaveRightCurrent;
    private Filter masterLeftCurrent;
    private Filter slaveLeftCurrent;
    private Filter masterRightTemp;
    private Filter slaveRightTemp;
    private Filter masterLeftTemp;
    private Filter slaveLeftTemp;

    private double delta;
    private int tempWarningCount;

    private SimpleMotorFeedforward m_ff_left,m_ff_right;
    private PIDController m_wpi_left_controller;
    private PIDController m_wpi_right_controller;

    private double kMaxControllerOutput[] = {3,3}; 
    private double gearBoxRatio = 10.71;
    private double wheelCircumference = 24 * .0254;  // 8 inch wheels
    private double kConversionFactor[] = {
        wheelCircumference/gearBoxRatio,
        wheelCircumference/gearBoxRatio};  // set to provide measurement in meters per motor revolution

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

        for (StormSpark s: getMotors()) {
            s.restoreFactoryDefaults();
            s.setSmartCurrentLimit(kSmartCurrentLimit);
            s.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        masterLeft.setInverted(kLeftSideInverted);
        slaveLeft.setInverted(kLeftSideInverted);
        masterRight.setInverted(kRightSideInverted);
        slaveRight.setInverted(kRightSideInverted);

        slaveLeft.follow(masterLeft);
        slaveRight.follow(masterRight);

        //  Configure encoders for meters/s
        masterLeft.getEncoder().setPositionConversionFactor(kConversionFactor[0]);
        masterRight.getEncoder().setPositionConversionFactor(kConversionFactor[1]);
        masterLeft.getEncoder().setVelocityConversionFactor(kConversionFactor[0]/60d);
        masterRight.getEncoder().setVelocityConversionFactor(kConversionFactor[1]/60d);

    }

    protected void setupTempControl() {
        delta = max(kTemperatureRampLimit - kTemperatureRampThreshold, 1.0); // Safety margin - don't want divide by 0!

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

    protected StormSpark[] getMotors()     {
        return new StormSpark[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive RightM Current", masterRightCurrent.update());
        SmartDashboard.putNumber("Drive RightS Current", slaveRightCurrent.update());
        SmartDashboard.putNumber("Drive LeftM Current", masterLeftCurrent.update());
        SmartDashboard.putNumber("Drive LeftS Current", slaveLeftCurrent.update());

        SmartDashboard.putNumber("Drive RightM Temp", masterRightTemp.update());
        SmartDashboard.putNumber("Drive RightS Temp", slaveRightTemp.update());
        SmartDashboard.putNumber("Drive LeftM Temp", masterLeftTemp.update());
        SmartDashboard.putNumber("Drive LeftS Temp", slaveLeftTemp.update());

        double temp = max(max(masterRight.getMotorTemperature(), masterLeft.getMotorTemperature()),
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
        // Using WPILib pid controller.  The command must have and manage the trapezoid object.  The drive 
        // subsystem doesn't know about it and expects the command to pass position and velocity setpoints
        double kV[] = {2.3,2.3}; // R2D2 on stand, voltage output 
        double kP[] = {45,45}; 
        //double kI[] = {1e-6,1e-6};
        double kI[] = {0,0};
        double kD[] = {0,0};

        m_wpi_left_controller = new PIDController(kP[0],kI[0],kD[0]);
        m_wpi_right_controller = new PIDController(kP[1],kI[1],kD[1]);

        m_ff_left = new SimpleMotorFeedforward(0,kV[0]);
        m_ff_right = new SimpleMotorFeedforward(0,kV[1]);
    }

    // Engage the PID controllers to move the robot to the incremental setpoint
    public void setPositionReference(double setPoint) {
        setPositionReferenceWithVelocity(setPoint,0);
    }

    // Engage the PID controllers to move the robot to the incremental setpoint (use velocity for feed forward)
    public void setPositionReferenceWithVelocity(double setPoint,double velocity) {
        // Get PID output 
        double left_out = MathUtil.clamp(m_wpi_left_controller.calculate(masterLeft.getEncoder().getPosition(), setPoint),-kMaxControllerOutput[0],kMaxControllerOutput[0]);
        double right_out = MathUtil.clamp(m_wpi_right_controller.calculate(masterRight.getEncoder().getPosition(), setPoint),-kMaxControllerOutput[1],kMaxControllerOutput[1]);

        SmartDashboard.putNumber("Drive Position Left PID output", left_out);
        SmartDashboard.putNumber("Drive Position Rifgt PID output", right_out);

        // Get Feedforward
        double ff_left_out = m_ff_left.calculate(velocity);
        double ff_right_out = m_ff_right.calculate(velocity);

        // Apply to motors
        masterLeft.setVoltage(left_out + ff_left_out);
        masterRight.setVoltage(right_out + ff_right_out);
    
        SmartDashboard.putNumber("Drive Position Target", setPoint);
    }

    public void resetPosition() {
        masterLeft.getEncoder().setPosition(0); 
        masterRight.getEncoder().setPosition(0);
    }

    public double getDistance() {
        return((masterLeft.getEncoder().getPosition() + masterRight.getEncoder().getPosition()) / 2);
    }

    public double getVelocity() {
        return((masterLeft.getEncoder().getVelocity() + masterRight.getEncoder().getVelocity()) / 2);
    }
}
