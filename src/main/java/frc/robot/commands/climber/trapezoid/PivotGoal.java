package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public enum PivotGoal {
  MOST_BACK(0),
  FIRST(58),
  SECOND(300),
  FURTHEST(300),
  Custom(0);

  TrapezoidProfile.State state;

  PivotGoal(double state) {
    this.state = new TrapezoidProfile.State(state, 0);
  }

  public TrapezoidProfile.State getState() {
    return state;
  }

  public static PivotGoal setCustom(double position) {
    Custom.state.position = position;
    return Custom;
  }
}
