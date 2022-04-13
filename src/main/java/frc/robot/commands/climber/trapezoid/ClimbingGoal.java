package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.kClimberRotationsPerUnitLength;

public enum ClimbingGoal {
  LOWEST(-.15),
  CLEARANCE_HEIGHT(75d / kClimberRotationsPerUnitLength),
  HIGHEST(.68),
  Custom(0);

  TrapezoidProfile.State state;

  ClimbingGoal(double state) {
    this.state = new TrapezoidProfile.State(state, 0);
  }

  public TrapezoidProfile.State getState() {
    return state;
  }

  public static ClimbingGoal setCustom(double position) {
    Custom.state.position = position;
    return Custom;
  }
}
