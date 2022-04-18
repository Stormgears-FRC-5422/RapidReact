package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public enum PivotGoal {
  MOST_BACK(Constants.kPivotForwardLimit),
  FIRST(0.044),
  SECOND(.25),
  FURTHEST(Constants.kPivotReverseLimit);

  TrapezoidProfile.State state;

  PivotGoal(double state) {
    this.state = new TrapezoidProfile.State(state, 0);
  }

  public TrapezoidProfile.State getState() {
    return state;
  }
}
