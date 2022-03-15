package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.LRSpeeds;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Climber extends ClimberParentSystem {
  protected final PIDController leftPIDController = new PIDController(0, 0, 0);
  protected final PIDController rightPIDController = new PIDController(0, 0, 0);
  private final StormSpark leftClimber = new StormSpark(kClimberLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final StormSpark rightClimber = new StormSpark(kClimberRightId, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0.0686, 0);
  private final double kS = 0.24;

  public Climber() {
    super();

    leftClimber.setInverted(kClimberLeftInverted);
    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftClimber.getEncoder().setVelocityConversionFactor(1 / 60d);

    rightClimber.setInverted(kClimberRightInverted);
    rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightClimber.getEncoder().setVelocityConversionFactor(1 / 60d);

    // Optimistic - we need to zero if the robot has been off...
    enableLimits();

    Shuffleboard.getTab("Climber").add(this);
    Shuffleboard.getTab("Climber").add("leftPID", leftPIDController);
    Shuffleboard.getTab("Climber").add("rightPID", rightPIDController);

    Shuffleboard.getTab("Climber").addNumber("PIDOutput", () -> pidOutput);
    Shuffleboard.getTab("Climber").addNumber("FeedForwardOutputs", () -> feedForwardOutputs);
    Shuffleboard.getTab("Climber").addNumber("Combined", () -> pidOutput + feedForwardOutputs);
    Shuffleboard.getTab("Climber").addBoolean("setSpeed", () -> setSpeed);
  }

  @Override
  public void periodic() {
    if (setSpeed) {
      leftClimber.set(speeds.left());
      rightClimber.set(speeds.right());
    }
    SmartDashboard.putNumber("climber left current", leftClimber.getOutputCurrent());
    SmartDashboard.putNumber("climber right current", rightClimber.getOutputCurrent());
    SmartDashboard.putNumber("climber left position", leftClimber.getEncoder().getPosition());
    SmartDashboard.putNumber("climber right position", rightClimber.getEncoder().getPosition());
  }

  public void stop() {
    setSpeed(LRSpeeds.stop());
  }

  @Override
  public LRSpeeds getSpeed() {
    return new LRSpeeds(leftClimber.get(), rightClimber.get());
  }

  @Override
  public void setSpeed(LRSpeeds lrSpeed) {
    setSpeed = true;
    this.speeds = lrSpeed;
    SmartDashboard.putNumber("% out", speeds.left());
  }

  @Override
  public void zero() {
    leftClimber.getEncoder().setPosition(0.0);
    rightClimber.getEncoder().setPosition(0.0);

    setLimits(-20.0f, -200.0f, -20.0f, -200.0f);
    enableLimits();

    System.out.println("Climber.zero()");
  }

  @Override
  public void disableLimits() {
    leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    System.out.println("Climber.disableLimits()");
  }

  @Override
  public void enableLimits() {
    leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    System.out.println("Climber.enableLimits()");
  }

  @Override
  public void setLimits(
      float forwardLeft, float reverseLeft, float forwardRight, float reverseRight) {
    leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardLeft);
    leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseLeft);
    rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardRight);
    rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseRight);
    System.out.println("Climber.setLimits()");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("left position", this::leftPosition, null);
    builder.addDoubleProperty("right position", this::rightPosition, null);
  }

  @Override
  public double leftPosition() {
    return -leftClimber.getEncoder().getPosition();
  }

  @Override
  public double rightPosition() {
    return -rightClimber.getEncoder().getPosition();
  }

  @Override
  public void leftPID(State state) {
    setSpeed = false;
    this.pidOutput =
        MathUtil.clamp(leftPIDController.calculate(leftPosition(), state.position), -12, 12);
    this.feedForwardOutputs = feedforward.calculate(state.velocity, 0) + kS * Math.signum(state.position - leftPosition());
    leftClimber.setVoltage(-(pidOutput + feedForwardOutputs));
  }

  @Override
  public void rightPID(State state) {
    setSpeed = false;
    double pid =
        MathUtil.clamp(rightPIDController.calculate(rightPosition(), state.position), -12, 12);
    double feed = feedforward.calculate(state.velocity, 0) + kS * Math.signum(state.position - leftPosition());
    rightClimber.setVoltage(-(pid + feed));
  }


}
