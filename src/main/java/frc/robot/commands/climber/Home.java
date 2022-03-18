package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberParentSystem;

class Home extends CommandBase {
  ClimberParentSystem subsystem;

  public Home(ClimberParentSystem subsystem) {
    System.out.println("Home()");
    this.subsystem = subsystem;
    addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      System.out.println("Home.initialize()");
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
      System.out.println("Home.end( interrupted = " + interrupted + " )");
    System.out.println(subsystem.getName() + " is " + "homed");
      subsystem.enableLimits();
      if (!interrupted) subsystem.zero();
    }
}
