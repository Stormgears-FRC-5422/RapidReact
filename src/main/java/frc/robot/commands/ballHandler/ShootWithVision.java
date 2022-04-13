package frc.robot.commands.ballHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.Supplier;

@Log.Exclude
public class ShootWithVision extends Shoot {
    private final Supplier<Double> distance;
    @Log.Exclude
    private final Shooter shooter;
    @Log.Exclude
    private final Feeder feeder;

    public ShootWithVision(Shooter shooter, Feeder feeder ,Supplier<Double> distance) {
        super(feeder, shooter);
        this.shooter = shooter;
        this.feeder = feeder;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        super.execute();
        shooter.setSetpoint(metersToRPS(distance.get()));
    }

    @Override
    public void execute() {
        double rps = metersToRPS(distance.get());
        shooter.setSetpoint(rps);
        super.execute();
    }

    //TODO: INSERT FUNCTION HERE
    private double metersToRPS(double meters) {
        double feet = Units.metersToFeet(meters);
        return 41.1*Math.pow(feet, 0.247);
    }
}
