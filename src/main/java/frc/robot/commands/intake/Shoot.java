package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;

public class Shoot extends CommandBase {

    private final Feeder feeder;
    private final Shooter shooter;

    public Shoot(Feeder feeder, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;
        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        feeder.setLimit(true);
        shooter.on();
    }

    @Override
    public void execute() {
        feeder.setLimit(!shooter.isReady());
        shooter.on();
        feeder.on();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.off();
        feeder.off();
    }

}
