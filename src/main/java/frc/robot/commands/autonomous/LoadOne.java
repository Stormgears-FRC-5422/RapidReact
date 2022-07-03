package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.ballHandler.Load;

import java.util.function.BooleanSupplier;

public class LoadOne extends CommandBase {
  private final Load load;
  private final BooleanSupplier limitSensorTripped;

  LoadOne(Load load, BooleanSupplier limitSensorTripped) {
    this.load = load;
    this.limitSensorTripped = limitSensorTripped;
    for (Subsystem requirement : load.getRequirements()) {
      addRequirements(requirement);
    }
  }

  @Override
  public void initialize() {
    load.initialize();
  }

  @Override
  public void execute() {
    load.execute();
  }

  @Override
  public void end(boolean interrupted) {
    load.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return limitSensorTripped.getAsBoolean();
  }
}
