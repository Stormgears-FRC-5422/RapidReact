package frc.robot.commands.ballHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.utils.filters.MovingAverage;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class ShootWithVision extends CommandBase implements Loggable {
    private final BooleanSupplier hasTarget;
    private final DoubleSupplier distance;
    @Log.Exclude
    private final Shooter shooter;
    @Log.Exclude
    private final Shoot shoot;
    @Log(methodName = "getValue")
    private final MovingAverage setpointMovingAverage;
    private final DoubleSupplier area;
    private double magicConstant;
    private double shooterSetpointChangingThreshold;
    private DoubleArrayLogEntry shooterDistanceRPSLog;
    private boolean change = true;


    @Config.Command
    InstantCommand regularThreshold = new InstantCommand(() -> shooterSetpointChangingThreshold = kShooterSetpointChangingThreshold);
    @Config.Command
    InstantCommand highThreshold = new InstantCommand(() -> shooterSetpointChangingThreshold = 100);

    public ShootWithVision(
            Shooter shooter,
            Shoot shoot,
            BooleanSupplier hasTarget,
            DoubleSupplier distance,
            DoubleArrayLogEntry shooterDistanceRPSLog, DoubleSupplier area) {
        this.shooter = shooter;
        this.shoot = shoot;
        this.hasTarget = hasTarget;
        this.distance = distance;
        this.area = area;
        magicConstant = kMagicVisionConstant;
        shooterSetpointChangingThreshold = kShooterSetpointChangingThreshold;
        setpointMovingAverage = new MovingAverage(35);
        if (shooterDistanceRPSLog != null) this.shooterDistanceRPSLog = shooterDistanceRPSLog;
        for (Subsystem requirement : shoot.getRequirements()) {
            addRequirements(requirement);
        }
    }

    @Override
    public void initialize() {
        change = true;
        shoot.initialize();
    }

    @Override
    public void execute() {
        shoot.execute();
        updateSetpoint();
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

    private void updateSetpoint() {
        if (hasTarget.getAsBoolean()) {
            double distanceMeters = distance.getAsDouble() / magicConstant;
            double rps = metersToRPS(distanceMeters);
            double targetArea = area.getAsDouble();
            if (targetArea < 3) {
                if (change) {
                    setpointMovingAverage.force(rps);
                    change = false;
                } else setpointMovingAverage.update(rps);
            }
            try {
                System.out.println("Distance: " + distanceMeters + " and shooting @ " + rps + " rps");
                double[] logEntry = new double[]{distanceMeters, rps, targetArea, shooter.setpoint(), setpointMovingAverage.getValue()};
                shooterDistanceRPSLog.append(logEntry);
            } catch (Exception e) {
                e.printStackTrace();
            }
            if (change || Math.abs(shooter.setpoint() - setpointMovingAverage.getValue()) > shooterSetpointChangingThreshold) {
                shooter.setSetpoint(setpointMovingAverage.getValue());
            }
        } else {
            try {
                System.out.println("No recorded distance and shooting @ " + shooter.getSpeed() + " rps");
                double[] logEntry = new double[]{0, shooter.setpoint(), 0, shooter.setpoint(), setpointMovingAverage.getValue()};
                shooterDistanceRPSLog.append(logEntry);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Config(defaultValueNumeric = 1.7)
    public void setMagicConstant(double magicConstant) {
        this.magicConstant = magicConstant;
    }

    @Config(defaultValueNumeric = 1)
    public void setShooterSetpointChangingThreshold(double shooterSetpointChangingThreshold) {
        this.shooterSetpointChangingThreshold = shooterSetpointChangingThreshold;
    }


}
