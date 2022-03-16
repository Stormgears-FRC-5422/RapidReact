package frc.robot.commands.drive;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.drive.PathFollow;
import frc.robot.subsystems.drive.SparkDrive;

import java.io.IOException;
import java.nio.file.Path;

public class PathAuto extends CommandBase {
    SparkDrive drive;
    RamseteCommand command;
    Trajectory trajectory;
    RamseteController controller = new RamseteController();
    PathFollow pf = new PathFollow();
    String path = "paths/YourPath.wpilib.json";

    public PathAuto(SparkDrive drive){
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
                pf::getPose2d,
                new RamseteController(2.0,0.7),
                pf.getFf(),
                pf.getDifferentialDriveKinematics(),
                drive::getWheelSpeeds,
                pf.getPidLeft(),
                pf.getPidRight(),
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
