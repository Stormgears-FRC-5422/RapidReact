package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Load;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.commands.drive.DriveTurnProfile;
import frc.robot.subsystems.sensors.NavX;
import frc.utils.drive.StormDrive;

public class DoubleBallAuto extends SequentialCommandGroup {

    public DoubleBallAuto(Load load, Shoot shoot, StormDrive drive, NavX navX) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveDistanceProfile(1.5d, 2d,1d, drive),
                        new ProxyScheduleCommand(load).withTimeout(5)),
                new DriveTurnProfile(180, 180d, 180d, drive, navX),
                new ProxyScheduleCommand(shoot)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println(
                "AUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISHAUTONOMOUSFINISH");
    }
}
