package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.utils.drive.StormDrive;

public class Autonomous extends SequentialCommandGroup {

  public Autonomous(Load load, Shoot shoot, StormDrive drive) {
    LoadOne loadOne = new LoadOne(load, load.feeder::getAbsoluteLimit);
    ShootOne shootOne = new ShootOne(shoot, load.feeder::getAbsoluteLimit);
    addCommands(
        new DriveDistanceProfile(-0.5d, 5d, 10d, drive),
        new DriveDistanceProfile(0.5d, 5d, 10d, drive),
        loadOne,
        new DriveDistanceProfile(-1.65d, 3d, 2d, drive),
        shootOne,
        new DriveDistanceProfile(-2d, 3d, 2d, drive));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        "AUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISH");
  }
}
