package frc.robot.commands.ballHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.ballHandler.Shooter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Log.Exclude
public class ShootWithVision extends CommandBase implements Loggable {
    private final BooleanSupplier hasTarget;
    private final DoubleSupplier distance;
    @Log.Exclude
    private final Shooter shooter;
    @Log.Exclude
    private final Shoot shoot;
    private DoubleArrayLogEntry shooterDistanceRPSLog;

    @Config
    double magicConstant = 1.8;
    @Config
    double changingThreshhold = 10;


    public ShootWithVision(
            Shooter shooter, Shoot shoot, BooleanSupplier hasTarget, DoubleSupplier distance, DoubleArrayLogEntry shooterDistanceRPSLog) {
        this.shooter = shooter;
        this.shoot = shoot;
        this.hasTarget = hasTarget;
        this.distance = distance;
        if (shooterDistanceRPSLog != null) this.shooterDistanceRPSLog = shooterDistanceRPSLog;
        for (Subsystem requirement : shoot.getRequirements()) {
            addRequirements(requirement);
        }
    }

    @Override
    public void initialize() {
        shoot.initialize();
        updateSetpoint(true);
    }

    @Override
    public void execute() {
        shoot.execute();
        updateSetpoint(false);
    }

    @Override
    public void end(boolean interrupted) {
        shoot.end(interrupted);
    }

    private double metersToRPS(double meters) {
        double feet = Units.metersToFeet(meters);
        // 41.1x^0.247
        return 41.1 * Math.pow(feet, 0.247);
    }

    private void updateSetpoint(boolean change) {
        if (hasTarget.getAsBoolean()) {
            //TODO Constants
            double distanceMeters = distance.getAsDouble() / magicConstant;
            double rps = metersToRPS(distanceMeters);
            //TODO Constants
            if(change ){//|| Math.abs(shooter.setpoint() - rps) > changingThreshhold) {
                shooter.setSetpoint(rps);
            }
            try {
                System.out.println("Distance: " + distanceMeters + " and shooting @ " + rps + " rps");
                double[] logEntry = new double[]{distanceMeters, rps};
                shooterDistanceRPSLog.append(logEntry);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else
            try {
                System.out.println("No recorded distance and shooting @ " + shooter.getSpeed() + " rps");
                double[] logEntry = new double[]{0, shooter.getSpeed()};
                shooterDistanceRPSLog.append(logEntry);
            } catch (Exception e) {
                e.printStackTrace();
            }
    }
}
