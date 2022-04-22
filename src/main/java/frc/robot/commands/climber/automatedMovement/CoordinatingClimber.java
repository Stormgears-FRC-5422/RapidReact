package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import frc.robot.subsystems.climber.HangerConstraints;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.kCoordinatingClimberRestTime;
import static frc.robot.Constants.kCoordingatingClimberEndPosition;

public class CoordinatingClimber extends CommandBase {
  ClimbingSubsystem climber;
  ClimbingSubsystem pivot;
  StormXboxController joystick;
  TrapezoidProfile.State climberState;
  TrapezoidProfile.State pivotState;
  LRSpeeds moveDown = new LRSpeeds(1, 1);
  Timer timer = new Timer();

  public CoordinatingClimber(ClimbingSubsystem climber, ClimbingSubsystem pivot) {
    this.climber = climber;
    this.pivot = pivot;

    addRequirements(climber, pivot);
    climberState = new TrapezoidProfile.State(0, 0);
    pivotState = new TrapezoidProfile.State(0, 0);
  }

  @Override
  public void initialize() {
    System.out.println("CoordinatingClimber.initialize()");
    pivot.resetPID();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(kCoordinatingClimberRestTime)) climber.setSpeed(moveDown);
    else climber.setSpeed(new LRSpeeds(0, 0));
    pivotState.position = HangerConstraints.getPivotPosition(climber.leftPosition());
    pivot.leftPID(pivotState);
    pivot.rightPID(pivotState);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
    timer.stop();
    climber.stop();
    pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return climber.leftPosition() < kCoordingatingClimberEndPosition
        && climber.rightPosition() < kCoordingatingClimberEndPosition;
  }
}
