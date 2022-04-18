package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import frc.robot.subsystems.climber.HangerConstraints;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

public class CoordinatingClimber extends CommandBase {
  ClimbingSubsystem climber;
  ClimbingSubsystem pivot;
  StormXboxController joystick;
  double leftClimberPosition;
  TrapezoidProfile.State climberState;
  TrapezoidProfile.State pivotState;
  LRSpeeds moveDown = new LRSpeeds(1, 1);
  Timer timer = new Timer();

  public CoordinatingClimber(
      ClimbingSubsystem climber, ClimbingSubsystem pivot, StormXboxController joystick) {
    this.climber = climber;
    this.pivot = pivot;
    this.joystick = joystick;

    addRequirements(climber, pivot);
    climberState = new TrapezoidProfile.State(0, 0);
    pivotState = new TrapezoidProfile.State(0, 0);
  }

  @Override
  public void initialize() {
    System.out.println("CoordinatingClimber.initialize()");
    leftClimberPosition = climber.leftPosition();
    pivot.resetPID();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    // We don't want the position to drift if there is no joystick input -
    // in that case just use the previous value (rather than the current one)
    //    if (joyVal != 0) {
    //      //            leftClimberPosition = climber.leftPosition() + 40 * copySign(1.0, joyVal);
    //      climber.setSpeed(new LRSpeeds(1 * copySign(1d, -joyVal), 1 * copySign(1d, -joyVal)));
    //    } else climber.stop();
    if (timer.hasElapsed(0.5)) {
      climber.setSpeed(moveDown);
      pivotState.position = HangerConstraints.getPivotPosition(climber.leftPosition());
    } else {
      climber.setSpeed(new LRSpeeds(0, 0));
      pivotState.position = HangerConstraints.getPivotPosition(leftClimberPosition);
    }
    pivot.leftPID(pivotState);
    pivot.rightPID(pivotState);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
    climber.stop();
    pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return climber.leftPosition() < Constants.kClimberMidpoint
        && climber.rightPosition() < Constants.kClimberMidpoint;
  }
}
