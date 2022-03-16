package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SparkDrive;

import java.io.IOException;
import java.nio.file.Path;

public class PathAuto extends CommandBase {
    SparkDrive drive = new SparkDrive();
    Trajectory trajectory =  new Trajectory();
    String path = "paths/YourPath.wpilib.json";

    public PathAuto(SparkDrive drive){
        this.drive=drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        super.initialize();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("error");
        }
    }

    @Override
    public void execute(){
        super.execute();
    }

}
