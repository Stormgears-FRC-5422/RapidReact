package frc.robot.commands.ballHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ballHandler.Shooter;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Log.Exclude
public class ShootWithVision extends CommandBase {
  private final BooleanSupplier hasTarget;
  private final DoubleSupplier distance;
  @Log.Exclude private final Shooter shooter;
  @Log.Exclude private final Shoot shoot;

  public ShootWithVision(
      Shooter shooter, Shoot shoot, BooleanSupplier hasTarget, DoubleSupplier distance) {
    this.shooter = shooter;
    this.shoot = shoot;
    this.hasTarget = hasTarget;
    this.distance = distance;
    for (Subsystem requirement : shoot.getRequirements()) {
      addRequirements(requirement);
    }
  }

  @Override
  public void initialize() {
    shoot.initialize();
    // TODO log distance to file w/ Timestamp
    double distanceToHub = distance.getAsDouble();
    double visionRPS = metersToRPS(distanceToHub);
    if (hasTarget.getAsBoolean()) shooter.setSetpoint(visionRPS);
    System.out.println(distanceToHub + " meters @ shooting at " + visionRPS + " rps");
  }

  @Override
  public void execute() {
    shoot.execute();
  }

  @Override
  public void end(boolean interrupted) {
    shoot.end(interrupted);
  }

  // TODO: INSERT FUNCTION HERE
  private double metersToRPS(double meters) {
    double feet = Units.metersToFeet(meters);
    // 41.1x^0.247
    return 41.1 * Math.pow(feet, 0.247);
  }
}
