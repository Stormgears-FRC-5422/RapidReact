package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public enum PivotGoal {
  MOST_BACK(5),
  FIRST(52),
  //    SECOND(150),
  FURTHEST(145);

  TrapezoidProfile.State state;

  PivotGoal(double state) {
    this.state = new TrapezoidProfile.State(state, 0);
  }

  public TrapezoidProfile.State getState() {
    return state;
  }
}
