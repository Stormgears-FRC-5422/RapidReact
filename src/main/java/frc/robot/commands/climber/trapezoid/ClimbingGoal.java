package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public enum ClimbingGoal {
  LOWEST(-0.05),
  CLEARANCE_HEIGHT(75d / Constants.kClimberRotationsPerUnitLength),
  HIGHEST(.65),
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
