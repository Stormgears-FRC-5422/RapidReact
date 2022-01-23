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

public class Drive extends SubsystemBase {
  private final StormMotorType motorType;
  private DifferentialDrive differentialDrive = null;


  public Drive() {
    motorType = StormMotor.motorType();
    differentialDrive = getDifferentialDrive(motorType);
    differentialDrive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public DifferentialDrive getDifferentialDrive(StormMotorType motorType){
    if (differentialDrive != null) return differentialDrive;
    if (motorType == StormMotorType.SPARK){
      CANSparkMax leftMaster = new CANSparkMax(FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax rightMaster = new CANSparkMax(FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax leftSlave = new CANSparkMax(REAR_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax rightSlave = new CANSparkMax(REAR_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

      leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
      leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);

      leftSlave.follow(leftMaster);
      rightSlave.follow(rightMaster);

      leftMaster.setInverted(LEFT_SIDE_INVERTED);
      leftSlave.setInverted(LEFT_SIDE_INVERTED);
      rightMaster.setInverted(RIGHT_SIDE_INVERTED);
      rightSlave.setInverted(RIGHT_SIDE_INVERTED);

      differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
      return differentialDrive;
    } else {
      if (motorType == StormMotorType.TALON) {
        WPI_TalonSRX leftMaster = new WPI_TalonSRX(FRONT_LEFT_ID);
        WPI_TalonSRX rightMaster = new WPI_TalonSRX(FRONT_RIGHT_ID);
        WPI_TalonSRX leftSlave = new WPI_TalonSRX(REAR_LEFT_ID);
        WPI_TalonSRX rightSlave = new WPI_TalonSRX(REAR_RIGHT_ID);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);

        leftMaster.setInverted(LEFT_SIDE_INVERTED);
        leftSlave.setInverted(LEFT_SIDE_INVERTED);
        rightMaster.setInverted(RIGHT_SIDE_INVERTED);
        rightSlave.setInverted(RIGHT_SIDE_INVERTED);

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        return differentialDrive;
      }
    }
    return null;
  }

  public StormMotorType getMotorType() {
    return motorType;
  }
}
