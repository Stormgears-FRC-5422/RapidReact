package frc.robot.commands.ballHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;

public class Load extends CommandBase {

  private final Intake intake;
  public final Feeder feeder;

  public Load(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {
    feeder.setLimit(true);
  }

  @Override
  public void execute() {
    intake.on();
    feeder.on();
  }

  @Override
  public void end(boolean interrupted) {
    intake.off();
    feeder.off();
  }
}
