package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;

import static frc.robot.commands.climber.automatedMovement.ClimbingGoal.*;
import static frc.robot.commands.climber.automatedMovement.PivotGoal.*;

public class ClimbingSequences {
  public final PositionClimber highestClimber;
  public final SequentialCommandGroup chinUp;
  public final SequentialCommandGroup autoClimb;
  public final SequentialCommandGroup backbend;
  public final CoordinatingClimber coordinatingClimber;

  public ClimbingSequences(Climber climber, Pivot pivot) {
    coordinatingClimber = new CoordinatingClimber(climber, pivot);
    highestClimber = new PositionClimber(climber, HIGHEST.getState());
    chinUp =
        new SequentialCommandGroup(
            new PositionClimber(climber, CLEARANCE_HEIGHT.getState()),
            new PositionPivot(pivot, MOST_BACK.getState()),
            new PositionClimber(climber, LOWEST.getState()),
            new PositionPivot(pivot, FIRST.getState()),
            new PositionClimber(climber, CLEARANCE_HEIGHT.getState()));
    backbend =
        new SequentialCommandGroup(
            new PositionPivot(pivot, SECOND.getState()),
            new PositionClimber(climber, HIGHEST.getState()));
    autoClimb =
        new SequentialCommandGroup(
            new ProxyScheduleCommand(chinUp),
            new ProxyScheduleCommand(backbend),
            new ProxyScheduleCommand(coordinatingClimber),
            new ProxyScheduleCommand(chinUp),
            new ProxyScheduleCommand(backbend),
            new ProxyScheduleCommand(coordinatingClimber),
            new ProxyScheduleCommand(chinUp));
  }
}
