package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;

public class Climber extends ClimberParentSystem {

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.24, 0, 0.0686, 0);

  public Climber() {
    super(
        kClimberLeftId,
        kClimberRightId,
        kClimberLeftInverted,
        kClimberRightInverted,
        new PIDController(kLeftClimberP, kLeftClimberI, kLeftClimberD),
        new PIDController(kRightClimberP, kRightClimberI, kRightClimberD),
        kClimberHomeCurrentLimit,
        kClimberHomeSetSpeed);
  }

  @Override
  void setLimits() {
    setLimits(-kCLimberForwardLimit, -kCLimberReverseLimit);
  }

  @Override
  public double feedForward(double velocity) {
    return feedforward.calculate(velocity);
  }
}
