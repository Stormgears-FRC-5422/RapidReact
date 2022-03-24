package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.ballHandler.Shoot;

import java.util.function.BooleanSupplier;

public class ShootOne extends CommandBase {
  private final Shoot shoot;
  private final BooleanSupplier limitSensorTripped;

  ShootOne(Shoot shoot, BooleanSupplier limitSensorTripped) {
    this.shoot = shoot;
    this.limitSensorTripped = limitSensorTripped;

    for (Subsystem requirement : shoot.getRequirements()) {
      addRequirements(requirement);
    }
  }

  @Override
  public void initialize() {
    shoot.initialize();
  }

  @Override
  public void execute() {
    shoot.execute();
  }

  @Override
  public void end(boolean interrupted) {
    shoot.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return !limitSensorTripped.getAsBoolean();
  }
}
