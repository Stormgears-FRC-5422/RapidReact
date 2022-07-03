package frc.robot.commands.climber.home;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class Home extends CommandBase {
  @Log.Exclude ClimbingSubsystem subsystem;

  public Home(ClimbingSubsystem subsystem) {
    System.out.println("Home()");
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Home.initialize()");
    subsystem.disableAllLimits();
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
    System.out.println(subsystem.getName() + " is homed");
    subsystem.enableAllLimits();
    if (!interrupted) subsystem.zero();
  }
}
