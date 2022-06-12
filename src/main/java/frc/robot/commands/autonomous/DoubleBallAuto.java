package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.ShootWithVision;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.commands.drive.DriveWithVision;
import frc.robot.subsystems.sensors.Vision;
import frc.utils.drive.StormDrive;

public class DoubleBallAuto extends SequentialCommandGroup {

  // TODO verify
  public DoubleBallAuto(
      Load load, StormDrive drive, Vision vision, ShootWithVision shootWithVision) {
    addCommands(
        new ParallelCommandGroup(
            new DriveDistanceProfile(1.5d, 2d, 1d, drive),
            new ProxyScheduleCommand(load).withTimeout(5)),
        new DriveWithVision(drive, () -> 0, () -> 0.5, vision).withTimeout(3.5),
        new ProxyScheduleCommand(shootWithVision));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        "AUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISH");
  }
}
