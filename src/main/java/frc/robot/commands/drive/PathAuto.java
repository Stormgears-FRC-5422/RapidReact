package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.drive.SparkDrive;
import frc.robot.subsystems.sensors.NavX;

import java.io.IOException;
import java.nio.file.Path;

public class PathAuto extends CommandBase {
    final SparkDrive drive;
    final RamseteCommand command;
    Trajectory trajectory;
    final NavX navX;
    Pose2d pose2d;
    final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.58);
    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.3,3,0);
//    final PIDController pidLeft = new PIDController(2.1514,0,0);
    final PIDController pidLeft = new PIDController(.21514,0,0);
//    final PIDController pidRight = new PIDController(4.7757,0,0);
    final PIDController pidRight = new PIDController(.47757,0,0);
    String path = "/home/lvuser/deploy/paths.json";
    final DifferentialDriveOdometry odometry;

    public PathAuto(SparkDrive drive, NavX navX){
        this.drive=drive;
        this.navX = navX;
        addRequirements(drive);
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("error");
        }
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navX.getHeading()));
        command = new RamseteCommand(
                trajectory,
                this::getPose2d,
                new RamseteController(2.0,0.7),
                ff,
                kinematics,
                this::getWheelSpeeds,
                pidLeft,
                pidRight,
                drive::setOutputRamsete,
                drive
        );
        Shuffleboard.getTab("PathFollowingShafflu").add(pidLeft);
        Shuffleboard.getTab("PathFollowingShafflu").add(pidRight);
        Shuffleboard.getTab("PathFollowingShafflu").addNumber("Left Velocity", drive::getLeftVelocity);
        Shuffleboard.getTab("PathFollowingShafflu").addNumber("Right Velocity", drive::getRightVelocity);
        Shuffleboard.getTab("PathFollowingShafflu").addNumber("Left Error", () -> pidLeft.getSetpoint() - drive.getLeftVelocity());
        Shuffleboard.getTab("PathFollowingShafflu").addNumber("Right Error", () -> pidRight.getSetpoint() - drive.getRightVelocity());
    }

    public Pose2d getPose2d(){
        return pose2d;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                (drive.getLeftVelocity()), /// 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60,
                (drive.getRightVelocity())); /// 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60

    }

    @Override
    public void initialize(){
        command.initialize();
    }

    @Override
    public void execute(){
        /*SmartDashboard.putNumber("",command.target);*/
        pose2d = odometry.update(
                Rotation2d.fromDegrees(navX.getHeading()),
                (drive.getLeftVelocity()), /// 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60,
                (drive.getRightVelocity()) /// 10.71 * StormProp.getNumber("wheelRadius",3.0) * 2 * Math.PI / 60
        );
        command.execute();
        //SmartDashboard.putNumber("pid left setpoint",pidLeft.getSetpoint());
    }

    @Override
    public void end(boolean interrupted){
        command.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }

}
