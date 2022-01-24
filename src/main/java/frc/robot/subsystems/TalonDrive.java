package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import utils.drive.Drive;
import utils.drive.StormMotorType;

import static frc.robot.Constants.*;
import static frc.robot.Constants.REAR_RIGHT_ID;

public class TalonDrive extends SubsystemBase implements Drive {
    private final DifferentialDrive differentialDrive;

    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(FRONT_LEFT_ID);
    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(FRONT_RIGHT_ID);
    private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(REAR_LEFT_ID);
    private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(REAR_RIGHT_ID);

    public TalonDrive() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftSlave.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);

        leftSlave.setInverted(LEFT_SIDE_INVERTED);
        leftMaster.setInverted(LEFT_SIDE_INVERTED);
        rightMaster.setInverted(RIGHT_SIDE_INVERTED);
        rightSlave.setInverted(RIGHT_SIDE_INVERTED);

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setSafetyEnabled(true);
    }

    @Override
    public StormMotorType motorType() {
        return StormMotorType.TALON;
    }

    @Override
    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    @Override
    public MotorController[] getMotors() {
        return new MotorController[]{leftMaster, rightMaster, leftSlave, leftMaster};
    }
}
