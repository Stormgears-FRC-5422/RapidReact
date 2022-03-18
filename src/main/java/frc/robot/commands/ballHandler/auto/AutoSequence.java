package frc.robot.commands.ballHandler.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.commands.drive.DriveDistanceProfile;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.utils.drive.StormDrive;

public class AutoSequence extends SequentialCommandGroup {
    public AutoSequence(Shoot shoot, StormDrive drive){
        addCommands(
                new ShootOne(shoot),
                new DriveDistanceProfile(-2d,2d,0.5d,drive));
    }


}
