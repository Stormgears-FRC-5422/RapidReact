package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.NavX;
import frc.utils.configfile.StormProp;

public class PathFollow extends SubsystemBase {
    SparkDrive sparkDrive = new SparkDrive();
    NavX nav = new NavX();
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(nav.getAngleDegrees()));
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.58);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0,0,0);
    PIDController pidLeft = new PIDController(0,0,0);
    PIDController pidRight = new PIDController(0,0,0);
    Pose2d pose2d;

    public DifferentialDriveKinematics getDifferentialDriveKinematics() {
        return kinematics;
    }
    public DifferentialDriveOdometry getDifferentialDriveOdometry(){
        return odometry;
    }
    public SimpleMotorFeedforward getFf(){
        return ff;
    }
    public PIDController getPidLeft(){
        return pidLeft;
    }
    public PIDController getPidRight(){
        return pidRight;
    }

    @Override
    public void periodic(){
        double[] encoders = sparkDrive.getEncoders();
        pose2d = odometry.update(
                Rotation2d.fromDegrees(nav.getAngleDegrees()),
                encoders[0] / 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60,
                encoders[1] / 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60
        );
    }


}
