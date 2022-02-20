// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class SparkDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;

    private final StormSpark masterLeft = new StormSpark(MASTER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(MASTER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(SLAVE_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(SLAVE_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

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
    //Set Position multiplier for actual encoder readout
//    masterLeft.getEncoder().setPositionConversionFactor(Constants.RATIO);
//    masterRight.getEncoder().setPositionConversionFactor(Constants.RATIO);
//    slaveLeft.getEncoder().setPositionConversionFactor(Constants.RATIO);
//    slaveRight.getEncoder().setPositionConversionFactor(Constants.RATIO);

    slaveLeft.follow(masterLeft);
    slaveRight.follow(masterRight);

    differentialDrive = new DifferentialDrive(masterLeft, masterRight);
        differentialDrive.setSafetyEnabled(true);
    }

    public double getDistance() {
        double distance = 0;
        StormSpark[] motors = getStormSpark();
        for (StormSpark motor:motors) {
            RelativeEncoder encoder = motor.getEncoder();
            double encoderDistance = encoder.getPosition();
            distance+=encoderDistance;
        }
        return distance;
    }

    public double calculateDriveVel(double goal) {
        double currDist = getDistance();
        return drivePID.calculate(currDist, goal);
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    protected MotorController[] getMotors()     {
        return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    private StormSpark[] getStormSpark() {
        return new StormSpark[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    public void ResetEncoders() {
        for (StormSpark spark: getStormSpark()) {
            spark.getEncoder().setPosition(0d);
        }
    }
}
