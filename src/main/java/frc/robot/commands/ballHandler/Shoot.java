package frc.robot.commands.ballHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.ballHandler.Shooter.Height;

public class Shoot extends PIDCommand implements Loggable {
  private final Feeder feeder;
  private final Shooter shooter;
  private final BooleanSupplier buttonPressed;
  //  @Config private final InstantCommand toggleShooter = new InstantCommand(this::toggleMode);
  @Config.PIDController(name = "Shooter Controller")
  private final PIDController pidController = getController();

  private Lights lights;
  @Log private boolean isReady;

  private double lastSpeed = 0;
  @Log private double speedDif = 0;

  public Shoot(Feeder feeder, Shooter shooter, BooleanSupplier buttonPressed, Lights lights) {
    super(
        new PIDController(kShooterP, kShooterI, kShooterD),
        shooter::getSpeed,
        shooter::setpoint,
        shooter::runToSpeed,
        shooter,
        feeder);
    this.feeder = feeder;
    this.shooter = shooter;
    this.buttonPressed = buttonPressed;
    if (lights != null) this.lights = lights;
  }

  @Override
  public void initialize() {
    feeder.setLimit(true);
//    shooter.resetExponential();
    lastSpeed = shooter.getSpeed();
    if (lights != null) lights.setShooting(true);
  }

  @Override
  public void execute() {
    super.execute();
    speedDif = Math.abs(lastSpeed - shooter.getSpeed());
    this.isReady = isReady(kShooterTolerance);
    feeder.setLimit(!(isReady && buttonPressed.getAsBoolean()));
    if (!speedWithin(kShooterkITolerance)) getController().reset();
    feeder.shootOn();
    lastSpeed = shooter.getSpeed();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.off();
    shooter.off();
    if (lights != null) lights.setShooting(false);
  }

  private boolean speedWithin(double percentTolerance) {
    return shooter.getSpeed() >= ((1 - (percentTolerance / 100)) * shooter.setpoint())
        && shooter.getSpeed() <= ((1 + (percentTolerance / 100)) * shooter.setpoint());
  }

  private boolean isReady(double percentTolerance) {
//    return shooter.getExponentialSpeed() >= ((1 - (percentTolerance / 100)) * shooter.setpoint())
//        && shooter.getExponentialSpeed() <= ((1 + (percentTolerance / 100)) * shooter.setpoint());
    return speedWithin(percentTolerance) && speedDif < 0.2;
  }

  public void toggleMode() {
    if (shooter.mode == Height.LOW) shooter.mode = Height.HIGH;
    else shooter.mode = Height.LOW;
  }

  public boolean getButtonPressed() {
    return buttonPressed.getAsBoolean();
  }
}
