package frc.robot.commands.ballHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ballHandler.Shooter;
import frc.utils.filters.MovingAverage;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.kMagicVisionConstant;
import static frc.robot.Constants.kShooterSetpointChangingThreshold;

public class ShootWithVision extends CommandBase implements Loggable {
  private final BooleanSupplier hasTarget;
  private final DoubleSupplier distance;
  @Log.Exclude private final Shooter shooter;
  @Log.Exclude private final Shoot shoot;
  @Log(methodName = "getValue") private final MovingAverage setpointMovingAverage;
  private double magicConstant;
  private double shooterSetpointChangingThreshold;
  private DoubleArrayLogEntry shooterDistanceRPSLog;

  public ShootWithVision(
      Shooter shooter,
      Shoot shoot,
      BooleanSupplier hasTarget,
      DoubleSupplier distance,
      DoubleArrayLogEntry shooterDistanceRPSLog) {
    this.shooter = shooter;
    this.shoot = shoot;
    this.hasTarget = hasTarget;
    this.distance = distance;
    magicConstant = kMagicVisionConstant;
    shooterSetpointChangingThreshold = kShooterSetpointChangingThreshold;
    setpointMovingAverage = new MovingAverage(25);
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
      double distanceMeters = distance.getAsDouble() / magicConstant;
      double rps = metersToRPS(distanceMeters);
      if (change) setpointMovingAverage.force(rps);
      else setpointMovingAverage.update(rps);
      // TODO for moving average change rps on next two lines to movingAverage.getValue() & change
      // threshold to (0.5)?
      // For now, just logging the moving average can change for matches 6-10
      if (change || Math.abs(shooter.setpoint() - rps) > shooterSetpointChangingThreshold) {
        shooter.setSetpoint(rps);
        try {
          System.out.println("Distance: " + distanceMeters + " and shooting @ " + rps + " rps");
          double[] logEntry = new double[] {distanceMeters, rps, setpointMovingAverage.getValue()};
          shooterDistanceRPSLog.append(logEntry);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    } else {
      try {
        System.out.println("No recorded distance and shooting @ " + shooter.getSpeed() + " rps");
        double[] logEntry = new double[] {0, shooter.setpoint(), setpointMovingAverage.getValue()};
        shooterDistanceRPSLog.append(logEntry);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Config(defaultValueNumeric = 1.8)
  public void setMagicConstant(double magicConstant) {
    this.magicConstant = magicConstant;
  }

  @Config(defaultValueNumeric = 5)
  public void setShooterSetpointChangingThreshold(double shooterSetpointChangingThreshold) {
    this.shooterSetpointChangingThreshold = shooterSetpointChangingThreshold;
  }
}
