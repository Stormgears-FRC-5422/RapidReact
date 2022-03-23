package frc.robot.subsystems.climber;

import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

@Log.Exclude
public class Climber extends ClimbingSubsystem {

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.24, 0, 0.0686, 0);

  private final SparkMaxLimitSwitch leftReverseLimitSwitch;
  private final SparkMaxLimitSwitch leftForwardLimitSwitch;
  private final SparkMaxLimitSwitch rightReverseLimitSwitch;
  private final SparkMaxLimitSwitch rightForwardLimitSwitch;

  private final SparkMaxLimitSwitch[] allHardLimitSwitches;

  public Climber() {
    super(
        kClimberLeftId,
        kClimberRightId,
        kClimberLeftInverted,
        kClimberRightInverted,
        new PIDController(kLeftClimberP, kLeftClimberI, kLeftClimberD),
        new PIDController(kRightClimberP, kRightClimberI, kRightClimberD),
        kClimberHomeCurrentLimit,
        kClimberHomeSetSpeed,
        kClimberCushion,
        kClimberCushionFloor);

    leftReverseLimitSwitch =
        leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    leftForwardLimitSwitch =
        leftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightReverseLimitSwitch =
        rightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightForwardLimitSwitch =
        rightMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    allHardLimitSwitches =
        new SparkMaxLimitSwitch[] {
          rightReverseLimitSwitch,
          rightForwardLimitSwitch,
          leftReverseLimitSwitch,
          leftForwardLimitSwitch
        };
  }

  @Override
  void setSoftLimits() {
    setSoftLimits(-kCLimberForwardLimit, -kCLimberReverseLimit);
  }

  public void onlyHardLimits() {
    disableSoftLimits();
    allLimitsOn = false;
    System.out.println("Only Hard Limits");
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
        leftReverseLimitSwitch.enableLimitSwitch(false);
        leftForwardLimitSwitch.enableLimitSwitch(true);
      } else {
        leftReverseLimitSwitch.enableLimitSwitch(true);
        leftForwardLimitSwitch.enableLimitSwitch(false);
      }
      if (rightPosition() < kClimberMidpoint) {
        rightReverseLimitSwitch.enableLimitSwitch(false);
        rightForwardLimitSwitch.enableLimitSwitch(true);
      } else {
        rightReverseLimitSwitch.enableLimitSwitch(true);
        rightForwardLimitSwitch.enableLimitSwitch(false);
      }
    }

    if (goingHome) {
      leftHome = leftHome || leftReverseLimitSwitch.isPressed();
      rightHome = rightHome || rightReverseLimitSwitch.isPressed();
    }
    super.periodic();
  }

  @Override
  public boolean isHome() {
    boolean leftTriggered = leftReverseLimitSwitch.isPressed();
    boolean rightTriggered = rightReverseLimitSwitch.isPressed();
    if (leftTriggered && rightTriggered) System.out.println("BOTH LIMITS ARE " + leftTriggered);
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
    allLimitsOn = false;
  }

  @Override
  public void enableAllLimits() {
    enableSoftLimits();
    enableHardLimits();
    allLimitsOn = true;
  }

  public void enableHardLimits() {
    overrideLimits = false;
    leftReverseLimitSwitch.enableLimitSwitch(true);
    leftForwardLimitSwitch.enableLimitSwitch(true);
    rightForwardLimitSwitch.enableLimitSwitch(true);
    rightForwardLimitSwitch.enableLimitSwitch(true);
  }

  public void disableAllHardLimits() {
    overrideLimits = true;
    leftReverseLimitSwitch.enableLimitSwitch(false);
    leftForwardLimitSwitch.enableLimitSwitch(false);
    rightForwardLimitSwitch.enableLimitSwitch(false);
    rightForwardLimitSwitch.enableLimitSwitch(false);
  }

}