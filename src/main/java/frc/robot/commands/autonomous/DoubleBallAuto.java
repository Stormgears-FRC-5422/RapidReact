package frc.robot.commands.autonomous;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.ballHandler.ShootWithVision;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.commands.drive.DriveTurnProfile;
import frc.robot.commands.drive.DriveWithVision;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.robot.subsystems.sensors.NavX;
import frc.robot.subsystems.sensors.Vision;
import frc.utils.drive.StormDrive;

public class DoubleBallAuto extends SequentialCommandGroup {

  public DoubleBallAuto(
          Load load,
          StormDrive drive,
          Feeder feeder,
          Shooter shooter,
          Lights lights,
          NavX navX,
          Vision vision, DoubleArrayLogEntry shooterDistanceRPSLog) {
    addCommands(
        new ParallelCommandGroup(
            new DriveDistanceProfile(1.5d, 2d, 1d, drive),
            new ProxyScheduleCommand(load).withTimeout(5)),
        new DriveTurnProfile(180, 180d, 180d, drive, navX),
        new DriveWithVision(drive, () -> 0, () -> 0, vision).withTimeout(2),
        new ShootWithVision(
            shooter,
            new Shoot(feeder, shooter, lights),
            vision::hasTarget,
            vision::getDistance, shooterDistanceRPSLog));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        "AUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISH");
  }
}
