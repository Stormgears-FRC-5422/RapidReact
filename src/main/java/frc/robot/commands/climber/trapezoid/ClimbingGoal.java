package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public enum ClimbingGoal {
  LOWEST(10),
  SECOND(80),
  FIRST(275),
  HIGHEST(275),
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
