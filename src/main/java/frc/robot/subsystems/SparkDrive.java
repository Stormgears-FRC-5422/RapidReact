// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class SparkDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;

    private final StormSpark masterLeft = new StormSpark(MASTER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(MASTER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(SLAVE_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(SLAVE_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    public final SparkMaxPIDController m_left_controller = masterLeft.getPIDController();
    public final SparkMaxPIDController m_right_controller = masterRight.getPIDController();



    
    // PID coefficients -- FIXME move to config file
    // These are tuned to the robot so they are private to the drive subsystem.  Commands 
    // should not know about them (or care)
    private double kP[] = {5e-5,5e-5}; 
    private double kI[] = {1e-6,1e-6};
    private double kD[] = {0,0}; 
    private double kIz[] = {0,0}; 
    private double kFF[] = {0.000156,0.000156}; 
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
        
        masterLeft.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        slaveLeft.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        masterRight.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        slaveRight.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        
        masterLeft.setIdleMode(StormSpark.IdleMode.kCoast);
        masterRight.setIdleMode(StormSpark.IdleMode.kCoast);
        slaveLeft.setIdleMode(StormSpark.IdleMode.kCoast); 
        slaveRight.setIdleMode(StormSpark.IdleMode.kCoast);
        
        masterLeft.setInverted(LEFT_SIDE_INVERTED);
        slaveLeft.setInverted(LEFT_SIDE_INVERTED);
        masterRight.setInverted(RIGHT_SIDE_INVERTED);
        slaveRight.setInverted(RIGHT_SIDE_INVERTED);
        
        slaveLeft.follow(masterLeft);
        slaveRight.follow(masterRight);
        
        differentialDrive = new DifferentialDrive(masterLeft, masterRight);
        differentialDrive.setSafetyEnabled(true);

        // SmartMotion parameters (trapezoid) set from command
        // Command has to manage end position as relative or absolute.  It may reset encoder position
        // to make relative move commands work.
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

        // 
        masterLeft.getEncoder().setPositionConversionFactor(kConversionFactor[0]);
        masterRight.getEncoder().setPositionConversionFactor(kConversionFactor[1]);
        masterLeft.getEncoder().setVelocityConversionFactor(kConversionFactor[0]/60d);
        masterRight.getEncoder().setVelocityConversionFactor(kConversionFactor[1]/60d);
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    protected MotorController[] getMotors()     {
        return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    public void setPositionReference(double setPoint) {
        m_left_controller.setReference(setPoint,CANSparkMax.ControlType.kSmartMotion);
        m_right_controller.setReference(setPoint,CANSparkMax.ControlType.kSmartMotion);
    }

    public void resetPosition() {
        masterLeft.getEncoder().setPosition(0);
        masterRight.getEncoder().setPosition(0);
    }

    public void setMaxVelocity(double velocity) {
        m_left_controller.setSmartMotionMaxVelocity(velocity, 0);
        m_right_controller.setSmartMotionMaxVelocity(velocity, 0);
    }

    public void setMaxAccel(double acceleration) {
        m_left_controller.setSmartMotionMaxAccel(acceleration, 0);
        m_right_controller.setSmartMotionMaxAccel(acceleration, 0);
    }

    public double getDistance() {
        return((masterLeft.getEncoder().getPosition() + masterRight.getEncoder().getPosition()) / 2);
    }

    public double getVelocity() {
        return((masterLeft.getEncoder().getVelocity() + masterRight.getEncoder().getVelocity()) / 2);
    }

    @Override
    public void periodic() {
    }

    public void simulationPeriodic() {
    }

}
