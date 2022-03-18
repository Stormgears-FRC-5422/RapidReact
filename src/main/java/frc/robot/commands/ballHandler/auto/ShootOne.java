package frc.robot.commands.ballHandler.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ballHandler.Shoot;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;

public class ShootOne extends CommandBase {

    final Shoot shoot;

    public ShootOne(Shoot shoot) {
        this.shoot = shoot;
    }

    @Override
    public void initialize() {
        shoot.initialize();
    }

    @Override
    public void execute() {
        shoot.execute();
    }

    @Override
    public boolean isFinished() {
        return !shoot.feeder.getRawLimit();
    }

    @Override
    public void end(boolean interrupted) {
        shoot.end(interrupted);
    }
}
