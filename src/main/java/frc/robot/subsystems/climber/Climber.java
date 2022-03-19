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

  private final SparkMaxLimitSwitch[] allLimitSwitches;

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

    leftReverseLimitSwitch =
        leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    leftForwardLimitSwitch =
        leftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightReverseLimitSwitch =
        rightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightForwardLimitSwitch =
        rightMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    allLimitSwitches =
        new SparkMaxLimitSwitch[] {
          rightReverseLimitSwitch,
          rightForwardLimitSwitch,
          leftReverseLimitSwitch,
          leftForwardLimitSwitch
        };

    enableSoftLimits();
  }

  @Override
  void setSoftLimits() {
    setSoftLimits(-kCLimberForwardLimit, -kCLimberReverseLimit);
  }

  public void onlyHardLimits() {
    disableSoftLimits();
    enableHardLimits(true);
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
    enableHardLimits(false);
    allLimitsOn = false;
  }

  @Override
  public void enableAllLimits() {
    enableSoftLimits();
    enableHardLimits(true);
    allLimitsOn = true;
  }

  private void enableHardLimits(boolean enabled) {
    System.out.println("Enabled: " + enabled);
    for (SparkMaxLimitSwitch limitSwitch : allLimitSwitches) {
      limitSwitch.enableLimitSwitch(enabled);
      System.out.println(limitSwitch.isLimitSwitchEnabled());
    }
  }
}
