package frc.robot.commands.ballHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;

public class Reverse extends CommandBase {

  private final Feeder feeder;
  private final Intake intake;

  public Reverse(Intake intake, Feeder feeder) {
    this.feeder = feeder;
    this.intake = intake;
    addRequirements(feeder, intake);
  }

  @Override
  public void execute() {
    feeder.reverse();
    intake.reverse();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.off();
    intake.off();
  }
}
