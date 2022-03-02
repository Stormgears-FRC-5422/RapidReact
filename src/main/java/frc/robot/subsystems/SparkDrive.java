// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormSpark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.Constants.*;

public class SparkDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;
    private SimpleMotorFeedforward m_ff_left,m_ff_right;

    private final StormSpark masterLeft = new StormSpark(MASTER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(MASTER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(SLAVE_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(SLAVE_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    public final SparkMaxPIDController m_left_controller = masterLeft.getPIDController();
    public final SparkMaxPIDController m_right_controller = masterRight.getPIDController();

    private PIDController m_wpi_left_controller;
    private PIDController m_wpi_right_controller;
    private final boolean m_use_spark_control = false;

    
    // PID coefficients -- FIXME move to config file
    // These are tuned to the robot so they are private to the drive subsystem.  Commands 
    // should not know about them (or care)
    private double kV[] = {0.19,0.19}; // R2D2 on stand 
    private double kMaxOutput[] = {1,1}; 
    private double kMinOutput[] = {-1,-1};
    private double gearBoxRatio = 10.71;
    private double wheelCircumference = 24 * .0254;  // 8 inch wheels
    private double kConversionFactor[] = {
        wheelCircumference/gearBoxRatio,
        wheelCircumference/gearBoxRatio};  // set to provide measurement in meters per motor revolution

    public SparkDrive() {
        masterLeft.restoreFactoryDefaults();
        slaveLeft.restoreFactoryDefaults();
        masterRight.restoreFactoryDefaults();
        slaveRight.restoreFactoryDefaults();
        
        // Set current limits
        masterLeft.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        slaveLeft.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        masterRight.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        slaveRight.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        
        // Configure Idle mode
        masterLeft.setIdleMode(StormSpark.IdleMode.kCoast);
        masterRight.setIdleMode(StormSpark.IdleMode.kCoast);
        slaveLeft.setIdleMode(StormSpark.IdleMode.kCoast); 
        slaveRight.setIdleMode(StormSpark.IdleMode.kCoast);
        
        // Configure Motor inversions
        masterLeft.setInverted(LEFT_SIDE_INVERTED);
        slaveLeft.setInverted(LEFT_SIDE_INVERTED);
        masterRight.setInverted(RIGHT_SIDE_INVERTED);
        slaveRight.setInverted(RIGHT_SIDE_INVERTED);

        // Configure Follower Mode
        slaveLeft.follow(masterLeft);
        slaveRight.follow(masterRight);
        
        // Create differential drive object
        differentialDrive = new DifferentialDrive(masterLeft, masterRight);
        differentialDrive.setSafetyEnabled(true);

        //  Configure encoders for meters/s
        masterLeft.getEncoder().setPositionConversionFactor(kConversionFactor[0]);
        masterRight.getEncoder().setPositionConversionFactor(kConversionFactor[1]);
        masterLeft.getEncoder().setVelocityConversionFactor(kConversionFactor[0]/60d);
        masterRight.getEncoder().setVelocityConversionFactor(kConversionFactor[1]/60d);

        // Setup PID controllers
        setup_controllers();
}

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    public void setCoastMode() {
        masterLeft.setIdleMode(StormSpark.IdleMode.kCoast);
        masterRight.setIdleMode(StormSpark.IdleMode.kCoast);
        slaveLeft.setIdleMode(StormSpark.IdleMode.kCoast); 
        slaveRight.setIdleMode(StormSpark.IdleMode.kCoast);

    }
    public void setBrakeMode() {
        masterLeft.setIdleMode(StormSpark.IdleMode.kBrake);
        masterRight.setIdleMode(StormSpark.IdleMode.kBrake);
        slaveLeft.setIdleMode(StormSpark.IdleMode.kBrake); 
        slaveRight.setIdleMode(StormSpark.IdleMode.kBrake);
    }

    protected MotorController[] getMotors()     {
        return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    // Engage the PID controllers to move the robot to the incremental setpoint
    public void setPositionReference(double setPoint) {
        setPositionReferenceWithVelocity(setPoint,0);
    }

    // Engage the PID controllers to move the robot to the incremental setpoint (use velocity for feed forward)
    public void setPositionReferenceWithVelocity(double setPoint,double velocity) {
        // velocity used for WPILib control. Spark controller already knows the velocity
        if (m_use_spark_control) {
            // SetReference on the Spark controller will engage the motors
            m_left_controller.setReference(setPoint,CANSparkMax.ControlType.kSmartMotion);
            m_right_controller.setReference(setPoint,CANSparkMax.ControlType.kSmartMotion);
        }
        else {
            // Get PID output 
            double left_out = m_wpi_left_controller.calculate(masterLeft.getEncoder().getPosition(), setPoint);
            double right_out = m_wpi_right_controller.calculate(masterRight.getEncoder().getPosition(), setPoint);

            // Get Feedforward
            double ff_left_out = m_ff_left.calculate(velocity);
            double ff_right_out = m_ff_right.calculate(velocity);

            // Apply to motors
            masterLeft.setVoltage(left_out + ff_left_out);
            masterRight.setVoltage(right_out + ff_right_out);
        }
        SmartDashboard.putNumber("Drive Position Target", setPoint);
    }

    public void resetPosition() {
        masterLeft.getEncoder().setPosition(0); 
        masterRight.getEncoder().setPosition(0);
    }

    public void setMaxVelocity(double velocity) {
        m_left_controller.setSmartMotionMaxVelocity(velocity, 0);
        m_right_controller.setSmartMotionMaxVelocity(velocity, 0);
        SmartDashboard.putNumber("Drive Max velocity", velocity);
    }

    public void setMaxAccel(double acceleration) {
        m_left_controller.setSmartMotionMaxAccel(acceleration, 0);
        m_right_controller.setSmartMotionMaxAccel(acceleration, 0);
        SmartDashboard.putNumber("Drive Max Acceleration", acceleration);
    }

    public double getDistance() {
        return((masterLeft.getEncoder().getPosition() + masterRight.getEncoder().getPosition()) / 2);
    }

    public double getVelocity() {
        return((masterLeft.getEncoder().getVelocity() + masterRight.getEncoder().getVelocity()) / 2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive Current Speed", masterLeft.getEncoder().getVelocity());
    }

    public void simulationPeriodic() {
    }

    private void setup_controllers() {
        if (m_use_spark_control)     {
            // SmartMotion parameters (trapezoid) set from command
            // Command has to manage end position as relative or absolute.  It may reset encoder position
            // to make relative move commands work.
            double kP[] = {.1,.1}; 
            double kI[] = {.000,.000};
            double kD[] = {0,0}; 
            double kIz[] = {0,0}; 
            double kFF[] = {0,0};

            m_left_controller.setP(kP[0]);
            m_left_controller.setI(kI[0]);
            m_left_controller.setD(kD[0]);
            m_left_controller.setIZone(kIz[0]);
            m_left_controller.setFF(kFF[0]);
            m_left_controller.setOutputRange(-1,1);

            m_right_controller.setP(kP[1]);
            m_right_controller.setI(kI[1]);
            m_right_controller.setD(kD[1]);
            m_right_controller.setIZone(kIz[1]);
            m_right_controller.setFF(kFF[1]);
            m_right_controller.setOutputRange(-1,1);

            m_left_controller.setSmartMotionAllowedClosedLoopError(0,0);
            m_right_controller.setSmartMotionAllowedClosedLoopError(0,0);
            m_left_controller.setSmartMotionMinOutputVelocity(0, 0);
            m_right_controller.setSmartMotionMinOutputVelocity(0, 0);
            m_left_controller.setIZone(0);
            m_right_controller.setIZone(0);
        }
        else {
            // Using WPILib pid controller.  The command must have and manage the trapezoid object.  The drive 
            // subsystem doesn't know about it and expects the command to pass position and velocity setpoints
            double kP[] = {5e-5,5e-5}; 
            double kI[] = {1e-6,1e-6};
            double kD[] = {0,0};

            m_wpi_left_controller = new PIDController(kP[0],kI[0],kD[0]);
            m_wpi_right_controller = new PIDController(kP[1],kI[1],kD[1]);

            m_ff_left = new SimpleMotorFeedforward(0,kV[0]);
            m_ff_right = new SimpleMotorFeedforward(0,kV[1]);

        }
    }

}
