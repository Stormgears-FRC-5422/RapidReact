package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.drive.StormDrive;

import java.util.Arrays;

import static frc.robot.Constants.*;

public class TalonDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;


    private final WPI_TalonSRX masterLeft = new WPI_TalonSRX(MASTER_LEFT_ID);
    private final WPI_TalonSRX masterRight = new WPI_TalonSRX(MASTER_RIGHT_ID);
    private final WPI_TalonSRX slaveLeft = new WPI_TalonSRX(SLAVE_LEFT_ID);
    private final WPI_TalonSRX slaveRight = new WPI_TalonSRX(SLAVE_RIGHT_ID);

    public TalonDrive() {
        slaveLeft.follow(masterLeft);
        slaveRight.follow(masterRight);

        slaveLeft.setNeutralMode(NeutralMode.Coast);
        masterRight.setNeutralMode(NeutralMode.Coast);
        masterLeft.setNeutralMode(NeutralMode.Coast);
        slaveRight.setNeutralMode(NeutralMode.Coast);

        slaveLeft.setInverted(LEFT_SIDE_INVERTED);
        masterLeft.setInverted(LEFT_SIDE_INVERTED);
        masterRight.setInverted(RIGHT_SIDE_INVERTED);
        slaveRight.setInverted(RIGHT_SIDE_INVERTED);

        differentialDrive = new DifferentialDrive(masterLeft, masterRight);
        differentialDrive.setSafetyEnabled(true);
    }

    @Override
    public double calculateDriveVel(double goal) {
        return 0d;
    }

    @Override
    public double calculateRotateVel(double goalRadians) {
        return 0d;
    }

    @Override
    public void ResetEncoders() {

    }

    @Override
    public double getDistance() {
        return 0;
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    protected MotorController[] getMotors() {
        return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }
}
