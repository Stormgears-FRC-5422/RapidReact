// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class SparkDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;

    private final StormSpark masterLeft = new StormSpark(kMasterLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(kMasterRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(kSlaveLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(kSlaveRightId, CANSparkMaxLowLevel.MotorType.kBrushless);

    public SparkDrive() {
        masterLeft.restoreFactoryDefaults();
        slaveLeft.restoreFactoryDefaults();
        masterRight.restoreFactoryDefaults();
        slaveRight.restoreFactoryDefaults();

        masterLeft.setSmartCurrentLimit(kSmartCurrentLimit);
        slaveLeft.setSmartCurrentLimit(kSmartCurrentLimit);
        masterRight.setSmartCurrentLimit(kSmartCurrentLimit);
        slaveRight.setSmartCurrentLimit(kSmartCurrentLimit);

        masterLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        masterRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        slaveLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        slaveRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        masterLeft.setInverted(kLeftSideInverted);
        slaveLeft.setInverted(kLeftSideInverted);
        masterRight.setInverted(kRightSideInverted);
        slaveRight.setInverted(kRightSideInverted);

        slaveLeft.follow(masterLeft);
        slaveRight.follow(masterRight);

        differentialDrive = new DifferentialDrive(masterLeft, masterRight);
            differentialDrive.setSafetyEnabled(true);
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    protected MotorController[] getMotors()     {
        return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }
}
