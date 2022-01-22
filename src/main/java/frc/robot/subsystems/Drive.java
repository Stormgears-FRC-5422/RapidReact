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
import utils.StormProp;

public class Drive extends SubsystemBase {


  public StormMotorType motorType;
  int frontLeftID = StormProp.getInt("frontLeftID", -1);
  int frontRightID = StormProp.getInt("frontRightID", -1);
  int rearRightID = StormProp.getInt("rearRightID", -1);
  int rearLeftID = StormProp.getInt("rearLeftID", -1);

  private DifferentialDrive differentialDrive = null;


  public Drive() {
    String motorTypeProp = StormProp.getString("stormMotorType", "Spark");
    if (motorTypeProp.equals("Spark")) motorType = StormMotorType.SPARK;
    else if (motorTypeProp.equals("Talon")) motorType = StormMotorType.TALON;
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
      CANSparkMax leftMaster = new CANSparkMax(frontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax rightMaster = new CANSparkMax(frontRightID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax leftSlave = new CANSparkMax(rearLeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
      CANSparkMax rightSlave = new CANSparkMax(rearRightID, CANSparkMaxLowLevel.MotorType.kBrushless);

      leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
      leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);

      leftSlave.follow(leftMaster);
      rightSlave.follow(rightMaster);
      differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
      return differentialDrive;
    } else {
      if (motorType == StormMotorType.TALON) {
        WPI_TalonSRX leftMaster = new WPI_TalonSRX(frontLeftID);
        WPI_TalonSRX rightMaster = new WPI_TalonSRX(frontRightID);
        WPI_TalonSRX leftSlave = new WPI_TalonSRX(rearLeftID);
        WPI_TalonSRX rightSlave = new WPI_TalonSRX(rearRightID);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        return differentialDrive;
      }
    }
    return null;
  }
}
