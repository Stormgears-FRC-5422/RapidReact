package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;

public class Pivot extends ClimbingSubsystem {
  private final ArmFeedforward feedforward = new ArmFeedforward(0.05, 0, 0.0686, 0);
  public Pivot() {
    super(
        kPivotLeftId,
        kPivotRightId,
        kPivotLeftInverted,
        kPivotRightInverted,
        new PIDController(kLeftPivotP, kLeftPivotI, kLeftPivotD),
        new PIDController(kRightPivotP, kRightPivotI, kRightPivotD),
        kPivotHomeCurrentLimit,
        kPivotHomeSetSpeed);
    }

  @Override
  public double feedForward(double velocity) {
    return feedforward.calculate(0, velocity);
    }

  @Override
  void setLimits() {
    setLimits(-kCLimberForwardLimit, -kCLimberReverseLimit);
  }
}

