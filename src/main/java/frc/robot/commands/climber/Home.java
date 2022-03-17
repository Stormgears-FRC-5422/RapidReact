package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberParentSystem;

public class Home extends CommandBase {
  ClimberParentSystem subsystem;

  public Home(ClimberParentSystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    subsystem.disableLimits();
    }

    @Override
    public void execute() {
    subsystem.goHome();
    }

    @Override
    public boolean isFinished() {
    return subsystem.isHome();
    }

    @Override
    public void end(boolean interrupted) {
    subsystem.enableLimits();
        if (!interrupted) {
            System.out.println("Not interrupted");
      subsystem.zero();
        }
    }
}
