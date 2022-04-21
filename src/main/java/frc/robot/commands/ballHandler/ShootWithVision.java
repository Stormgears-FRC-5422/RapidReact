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
  private boolean hasPressedButton;

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
    hasPressedButton = false;
    updateSetpoint();
  }

  @Override
  public void execute() {
    if (shoot.getButtonPressed()) updateSetpointOnButton();
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

  private void updateSetpointOnButton() {
    if (hasPressedButton) return;
    updateSetpoint();
    hasPressedButton = true;
  }

  private void updateSetpoint() {
    if (hasTarget.getAsBoolean()) {
      double distanceMeters = distance.getAsDouble() / 1.5;
      double rps = metersToRPS(distanceMeters);
      shooter.setSetpoint(rps);
      System.out.println("Distance: " + distanceMeters + " and shooting @ " + rps + " rps");
    }
    System.out.println("No recorded distance and shooting @ " + shooter.getSpeed() + " rps");
  }
}
