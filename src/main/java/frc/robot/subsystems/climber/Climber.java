package frc.robot.subsystems.climber;

import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

@Log.Exclude
public class Climber extends ClimbingSubsystem {

  private final ElevatorFeedforward feedforward;

  private final SparkMaxLimitSwitch leftReverseHardLimitSwitch;
  private final SparkMaxLimitSwitch leftForwardHardLimitSwitch;
  private final SparkMaxLimitSwitch rightReverseHardLimitSwitch;
  private final SparkMaxLimitSwitch rightForwardHardLimitSwitch;

  public Climber() {
    super(
        kClimberLeftId,
        kClimberRightId,
        kClimberLeftInverted,
        kClimberRightInverted,
        kClimberRotationsPerUnitLength,
        new PIDController(kLeftClimberP, kLeftClimberI, kLeftClimberD),
        new PIDController(kRightClimberP, kRightClimberI, kRightClimberD),
        kClimberHomeCurrentLimit,
        kClimberHomeSetSpeed,
        kClimberCushion,
        kClimberCushionFloor);

    feedforward = new ElevatorFeedforward(0.5, 0, kNeo550NominalVoltage / kClimberMaxVelocity, 0);

    leftReverseHardLimitSwitch =
        leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    leftForwardHardLimitSwitch =
        leftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightReverseHardLimitSwitch =
        rightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightForwardHardLimitSwitch =
        rightMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
  }

  @Override
  void setSoftLimits() {
    setSoftLimits(kClimberForwardLimit, kClimberReverseLimit);
  }

  public void onlyHardLimits() {
    disableSoftLimits();
  }

  @Override
  public void goHome() {
    onlyHardLimits();
    super.goHome();
  }

  @Override
  public void periodic() {

    if (hasBeenHomed && !overrideLimits) {
      if (leftPosition() < kClimberMidpoint) {
        leftReverseHardLimitSwitch.enableLimitSwitch(false);
        leftForwardHardLimitSwitch.enableLimitSwitch(true);
      } else {
        leftReverseHardLimitSwitch.enableLimitSwitch(true);
        leftForwardHardLimitSwitch.enableLimitSwitch(false);
      }
      if (rightPosition() < kClimberMidpoint) {
        rightReverseHardLimitSwitch.enableLimitSwitch(false);
        rightForwardHardLimitSwitch.enableLimitSwitch(true);
      } else {
        rightReverseHardLimitSwitch.enableLimitSwitch(true);
        rightForwardHardLimitSwitch.enableLimitSwitch(false);
      }
    }

    if (goingHome) {
      leftHome = leftHome || leftReverseHardLimitSwitch.isPressed();
      rightHome = rightHome || rightReverseHardLimitSwitch.isPressed();
    }
    super.periodic();
  }

  @Override
  public boolean isHome() {
    boolean leftTriggered = leftReverseHardLimitSwitch.isPressed();
    boolean rightTriggered = rightReverseHardLimitSwitch.isPressed();

    if (leftTriggered && rightTriggered) System.out.println("BOTH LIMITS ARE pressed");

    return (leftTriggered && rightTriggered) || super.isHome();
  }

  @Override
  public double feedForward(double velocity) {
    return feedforward.calculate(velocity);
  }

  @Override
  public void disableAllLimits() {
    disableSoftLimits();
    disableAllHardLimits();
  }

  @Override
  public void enableAllLimits() {
    enableSoftLimits();
    enableHardLimits();
  }

  public void enableHardLimits() {
    overrideLimits = false;
    // The rest should be automatic in periodic
  }

  public void disableAllHardLimits() {
    overrideLimits = true;
    leftReverseHardLimitSwitch.enableLimitSwitch(false);
    leftForwardHardLimitSwitch.enableLimitSwitch(false);
    rightForwardHardLimitSwitch.enableLimitSwitch(false);
    rightForwardHardLimitSwitch.enableLimitSwitch(false);
  }

  public boolean isReverseLimitTripped() {
    return (leftReverseHardLimitSwitch.isPressed() && rightReverseHardLimitSwitch.isPressed())
        && leftReverseHardLimitSwitch.isLimitSwitchEnabled()
        && rightReverseHardLimitSwitch.isLimitSwitchEnabled();
  }

  public boolean isForwardLimitTripped() {
    return (leftForwardHardLimitSwitch.isPressed() && rightForwardHardLimitSwitch.isPressed())
        && leftForwardHardLimitSwitch.isLimitSwitchEnabled()
        && rightForwardHardLimitSwitch.isLimitSwitchEnabled();
  }
}
