// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
import frc.utils.motorcontrol.StormSpark;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.drive.Drive;
import frc.utils.drive.StormMotorType;

import static frc.robot.Constants.*;

public class SparkDrive extends SubsystemBase /*implements Drive*/ {
    private final DifferentialDrive differentialDrive;
    private final StormSpark leftMaster = new StormSpark(LEFT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightMaster = new StormSpark(RIGHT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark leftSlave = new StormSpark(LEFT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightSlave = new StormSpark(RIGHT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    public SparkDrive() {
        System.out.println("In SparkDriveConstructor");

        leftMaster.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        rightSlave.restoreFactoryDefaults();

        leftMaster.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        leftSlave.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        rightMaster.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
        rightSlave.setSmartCurrentLimit(SMART_CURRENT_LIMIT);

        leftMaster.setIdleMode(StormSpark.IdleMode.kCoast);
        rightMaster.setIdleMode(StormSpark.IdleMode.kCoast);
        leftSlave.setIdleMode(StormSpark.IdleMode.kCoast);
        rightSlave.setIdleMode(StormSpark.IdleMode.kCoast);

        leftMaster.setInverted(LEFT_SIDE_INVERTED);
        leftSlave.setInverted(LEFT_SIDE_INVERTED);
        rightMaster.setInverted(RIGHT_SIDE_INVERTED);
        rightSlave.setInverted(RIGHT_SIDE_INVERTED);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setSafetyEnabled(true);
    }

//    //@Override
//    public MotorController[] getMotors() {
//        return new MotorController[]{leftMaster, rightMaster, leftSlave, rightSlave};
//    }

    @Override
    public void periodic() {
    }

    //@Override
    public void simulationPeriodic() {
    }

    // @Override
    // public StormMotorType motorType() {
    //     return StormMotorType.SPARK;
    // }

    // @Override
    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

}
