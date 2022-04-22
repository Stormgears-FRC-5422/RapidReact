package frc.robot.commands.ballHandler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;

import java.util.function.BooleanSupplier;

public class RunIdleShooter extends CommandBase {
    private final Shooter shooter;
    private final BooleanSupplier proximitySensor;

    public RunIdleShooter(Shooter shooter, BooleanSupplier proximitySensor){
        this.shooter = shooter;
        this.proximitySensor = proximitySensor;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if(proximitySensor.getAsBoolean()) shooter.halfSpeed();
        else shooter.off();
    }
}
