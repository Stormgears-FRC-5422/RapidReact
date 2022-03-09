package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormTalon;

import static frc.robot.Constants.*;

public class TalonDrive extends StormDrive {
    private final DifferentialDrive differentialDrive;

    private final StormTalon masterLeft = new StormTalon(kMasterLeftId);
    private final StormTalon masterRight = new StormTalon(kMasterRightId);
    private final StormTalon slaveLeft = new StormTalon(kSlaveLeftId);
    private final StormTalon slaveRight = new StormTalon(kSlaveRightId);

    public TalonDrive() {
      slaveLeft.follow(masterLeft);
      slaveRight.follow(masterRight);

      slaveLeft.setNeutralMode(NeutralMode.Coast);
      masterRight.setNeutralMode(NeutralMode.Coast);
      masterLeft.setNeutralMode(NeutralMode.Coast);
      slaveRight.setNeutralMode(NeutralMode.Coast);

      slaveLeft.setInverted(kLeftSideInverted);
      masterLeft.setInverted(kLeftSideInverted);
      masterRight.setInverted(kRightSideInverted);
      slaveRight.setInverted(kRightSideInverted);

        // The scale can't be > 1.0 - if that's what we're given, flip the sense by reducing
        // the other side of the drive
        if (kRightSideScale > 1.0) {
            masterLeft.setScale(1.0/kRightSideScale);
            slaveLeft.setScale(1.0/kRightSideScale);
        } else {
            masterRight.setScale(kRightSideScale);
            slaveRight.set(kRightSideScale);
        }

        differentialDrive = new DifferentialDrive(masterLeft, masterRight);
          differentialDrive.setSafetyEnabled(true);
    }

    public DifferentialDrive getDifferentialDrive() {
          return differentialDrive;
      }

    protected MotorController[] getMotors() {
      return new MotorController[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }
}
