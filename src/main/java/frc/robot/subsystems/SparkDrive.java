// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drive.TrapezoidalPIDRotate;
import frc.utils.drive.StormDrive;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class SparkDrive extends SubsystemBase {

    private final DifferentialDrive differentialDrive;
//    private final NavX navX;

    private final StormSpark masterLeft = new StormSpark(MASTER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark masterRight = new StormSpark(MASTER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveLeft = new StormSpark(SLAVE_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark slaveRight = new StormSpark(SLAVE_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    protected boolean reverse = false;
    protected boolean precision = false;

    //PID controller for drive
    private PIDController drivePID = new PIDController(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive);

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
        StormSpark[] motors = getStormSparks();
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

//    public double calculateRotateVel(double goalRadians) {
//        return rotatePID.calculate(navX.getAngle(), goalRadians);
//    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    private StormSpark[] getStormSparks() {
        return new StormSpark[] {masterLeft, masterRight, slaveLeft, slaveRight};
    }

    public void ResetEncoders() {
        for (StormSpark spark: getStormSparks()) {
            spark.getEncoder().setPosition(0d);
        }
    }

    public void rotate(double zRotation) {
        if (Math.abs(zRotation) > 1) {
            System.out.println("Not valid " + zRotation);
            zRotation = 1;
        }
        getDifferentialDrive().arcadeDrive(0, zRotation, false);
    }

    public boolean getReverse() {
        return this.reverse;
    }

    public void setReverse(boolean r) {
        this.reverse = r;
    }

    public void toggleReverse() {
        reverse = !reverse;
        System.out.println("reverse = " + reverse);
    }

    public void togglePrecision() {
        precision = !precision;
        System.out.println("precision = " + precision);
    }

    public boolean getPrecisions() {
        return this.precision;
    }

    public void setPrecision(boolean p) {
        this.precision = p;
    }
}
