package frc.robot.commands.climber.home;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;

public class HomeClimbingSystem extends ParallelCommandGroup {

  public HomeClimbingSystem(Climber climber, Pivot pivot) {
    super(new Home(climber), new Home(pivot));
  }
}
