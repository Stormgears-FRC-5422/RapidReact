// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import utils.drive.StormMotor;
import utils.drive.StormMotorType;

import static frc.robot.Constants.*;

public class SparkDrive extends SubsystemBase implements Drive{
  private DifferentialDrive differentialDrive;

  private CANSparkMax leftMaster = new CANSparkMax(FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightMaster = new CANSparkMax(FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSlave = new CANSparkMax(REAR_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSlave = new CANSparkMax(REAR_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);


  public SparkDrive() {

    leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);

    leftMaster.setInverted(LEFT_SIDE_INVERTED);
    leftSlave.setInverted(LEFT_SIDE_INVERTED);
    rightMaster.setInverted(RIGHT_SIDE_INVERTED);
    rightSlave.setInverted(RIGHT_SIDE_INVERTED);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
    differentialDrive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }


  @Override
  public StormMotorType motorType() {
    return StormMotorType.SPARK;
  }
}
