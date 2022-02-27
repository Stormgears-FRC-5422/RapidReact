// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.drive.StormDrive;
import frc.utils.filters.ExponentialAverage;
import frc.utils.filters.Filter;
import frc.utils.motorcontrol.StormSpark;

import java.util.Map;

import static edu.wpi.first.wpilibj.DriverStation.reportWarning;
import static frc.robot.Constants.*;
import static java.lang.Math.max;

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


    public SparkDrive() {
        setupMotors();
        setupTempControl();
        setupSlewFactors();

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

}
