package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.drive.SparkDrive;

import java.io.IOException;
import java.nio.file.Path;

public class PathAuto extends CommandBase {
    SparkDrive drive;
    RamseteCommand command;
    Trajectory trajectory;
    final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.58);
    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0,0,0);
    final PIDController pidLeft = new PIDController(0,0,0);
    final PIDController pidRight = new PIDController(0,0,0);
    String path = "/home/lvuser/deploy/paths.json";

    public PathAuto(SparkDrive drive){
        Shuffleboard.getTab("PathFollowingShafflu").add(pidLeft);
        Shuffleboard.getTab("PathFollowingShafflu").add(pidRight);
        this.drive=drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("error");
        }
        command = new RamseteCommand(
                trajectory,
                drive::getPose2d,
                new RamseteController(2.0,0.7),
                ff,
                kinematics,
                drive::getWheelSpeeds,
                pidLeft,
                pidRight,
                drive::setOutput,
                drive
                );
        command.initialize();
    }

    @Override
    public void execute(){
        command.execute();
    }

    public void end(boolean interrupted){
        command.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }

}
