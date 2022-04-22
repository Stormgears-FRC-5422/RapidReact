package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.utils.drive.StormDrive;

public class Autonomous extends SequentialCommandGroup {

  private final Load load;

  // TODO verify that this works
  public Autonomous(Load load, Shoot shoot, StormDrive drive) {
    this.load = load;
    LoadOne loadOne = new LoadOne(load, load.feeder::getAbsoluteLimit);
    CommandBase shoot1 = shoot.until(this::limit);
    addCommands(
        loadOne,
        new DriveDistanceProfile(-1.65d, 3d, 2d, drive),
        shoot1,
        new DriveDistanceProfile(-2d, 3d, 2d, drive));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        "AUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISH");
  }

  public boolean limit() {
    return !load.feeder.getAbsoluteLimit();
  }
}
