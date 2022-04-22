package frc.robot.commands.climber.automatedMovement;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.kClimberRotationsPerUnitLength;

public enum ClimbingGoal {
  LOWEST(-.1),
  CLEARANCE_HEIGHT(75d / kClimberRotationsPerUnitLength),
  HIGHEST(.75);

  TrapezoidProfile.State state;

  ClimbingGoal(double state) {
    this.state = new TrapezoidProfile.State(state, 0);
  }

  public TrapezoidProfile.State getState() {
    return state;
  }
}
