package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

@Log.Exclude
public class Pivot extends ClimbingSubsystem {
  //private final ArmFeedforward feedforward = new ArmFeedforward(0.05, 0, 0.0686, 0);
  private final ElevatorFeedforward feedforward;

  public Pivot() {
    super(
        kPivotLeftId,
        kPivotRightId,
        kPivotLeftInverted,
        kPivotRightInverted,
        kPivotRotationsPerUnitLength,
        new PIDController(kLeftPivotP, kLeftPivotI, kLeftPivotD),
        new PIDController(kRightPivotP, kRightPivotI, kRightPivotD),
        kPivotHomeCurrentLimit,
        kPivotHomeSetSpeed,
        kPivotCushion,
        kPivotCushionFloor);

    feedforward = new ElevatorFeedforward(0.5, 0, kNeo550NominalVoltage / kPivotMaxVelocity, 0);

  }

  @Override
  public void disableAllLimits() {
    overrideLimits = false;
    disableSoftLimits();
  }

  @Override
  public void enableAllLimits() {
    overrideLimits = true;
    enableSoftLimits();
  }

  @Override
  public double feedForward(double velocity) {
    return feedforward.calculate(velocity,0);
  }

  @Override
  void setSoftLimits() {
    setSoftLimits(kPivotForwardLimit, kPivotReverseLimit);
  }
}
