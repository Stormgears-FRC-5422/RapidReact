package frc.robot.commands.ballHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.ballHandler.Shooter.Height;

public class Shoot extends PIDCommand {
  private final Feeder feeder;
  private final Shooter shooter;
  //  @Config private final InstantCommand toggleShooter = new InstantCommand(this::toggleMode);
  @Config.PIDController(name = "Shooter Controller")
  private final PIDController pidController = getController();

  @Log private boolean isReady;

  public Shoot(Feeder feeder, Shooter shooter) {
    super(
        new PIDController(kShooterP, kShooterI, kShooterD),
        shooter::getSpeed,
        shooter::setpoint,
        shooter::runToSpeed,
        shooter,
        feeder);
    this.feeder = feeder;
    this.shooter = shooter;
    //        Shuffleboard.getTab("Shoot Command").add(this);
  }

  @Override
  public void initialize() {
    feeder.setLimit(true);
    shooter.resetExponential();
  }

  @Override
  public void execute() {
    super.execute();
    this.isReady = isReady(kShooterTolerance);
    feeder.setLimit(!isReady);
    if (!resetIntegral(kShooterkITolerance)) getController().reset();
    feeder.on();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.off();
    shooter.off();
  }

  private boolean resetIntegral(double percentTolerance) {
    return shooter.getSpeed() >= ((1 - (percentTolerance / 100)) * shooter.setpoint())
        && shooter.getSpeed() <= ((1 + (percentTolerance / 100)) * shooter.setpoint());
  }

  private boolean isReady(double percentTolerance) {
    return shooter.getExponentialSpeed() >= ((1 - (percentTolerance / 100)) * shooter.setpoint())
        && shooter.getExponentialSpeed() <= ((1 + (percentTolerance / 100)) * shooter.setpoint());
  }

  public void toggleMode() {
    if (shooter.mode == Height.LOW) shooter.mode = Height.HIGH;
    else shooter.mode = Height.LOW;
  }

  //    @Override
  //    public void initSendable(SendableBuilder builder) {
  //        builder.addDoubleProperty("I Value", getController()::getI, getController()::setI);
  //        builder.addDoubleProperty("P Value", getController()::getP, getController()::setP);
  //        builder.addDoubleProperty("D Value", getController()::getD, getController()::setD);
  //    builder.addBooleanProperty("limit", () -> resetIntegral(kShooterTolerance), null);
  //    }
}
