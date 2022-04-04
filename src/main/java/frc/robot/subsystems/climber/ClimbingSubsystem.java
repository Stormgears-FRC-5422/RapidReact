package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRSpeeds;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.MathUtil.clamp;
import static java.lang.Math.*;
import static frc.robot.Constants.*;

@Log.Exclude
public abstract class ClimbingSubsystem extends SubsystemBase implements Loggable {
  @Log(name = "lc", methodName = "update")
  protected final ExponentialAverage leftCurrent;

  @Log(name = "rc", methodName = "update")
  protected final ExponentialAverage rightCurrent;

  protected final StormSpark leftMotor;
  protected final StormSpark rightMotor;

  protected final String name = getName();
  @Config.PIDController protected final PIDController leftPIDController;
  @Config.PIDController protected final PIDController rightPIDController;

  protected final double rotationsPerUnitLength;
  protected final double homeCurrentLimit;
  protected final double homeSpeed;
  protected final double cushion;
  protected final double cushionFloor;

  @Log public boolean allLimitsOn;
  @Log.Exclude // TODO
  protected LRSpeeds speeds = new LRSpeeds();
  @Log protected boolean setSpeed = true;
  @Log protected boolean goingHome = false;
  @Log protected boolean leftHome = false;
  @Log protected boolean rightHome = false;
  @Log protected boolean hasBeenHomed = false;
  // For monitoring on Shuffleboard, does nothing
  @Log protected double pidOutput = 0;
  @Log protected double feedForwardOutputs = 0;

  @Log protected double forwardSoftLimit;
  @Log protected double reverseSoftLimit;

  @Log protected boolean overrideLimits = false;

  protected ClimbingSubsystem(
      int leftMotorID,
      int rightMotorID,
      boolean leftInverted,
      boolean rightInverted,
      double rotationsPerUnitLength,
      PIDController leftPIDController,
      PIDController rightPIDController,
      double homeCurrentLimit,
      double homeSpeed,
      double cushion,
      double cushionFloor) {
    leftMotor = new StormSpark(leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);
    rightMotor = new StormSpark(rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);

    this.leftCurrent = new ExponentialAverage(leftMotor::getOutputCurrent, 4);
    this.rightCurrent = new ExponentialAverage(rightMotor::getOutputCurrent, 4);

    this.leftPIDController = leftPIDController;
    this.rightPIDController = rightPIDController;

    //    leftPIDController.setTolerance(4);
    //    rightPIDController.setTolerance(4);

    this.rotationsPerUnitLength = rotationsPerUnitLength;
    this.homeCurrentLimit = homeCurrentLimit;
    this.homeSpeed = homeSpeed;
    this.cushion = cushion;
    this.cushionFloor = cushionFloor;


    leftMotor.setInverted(leftInverted);
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//    leftMotor.getEncoder().setVelocityConversionFactor(1 / 60d);
    leftMotor.setOpenLoopRampRate(0.25);

    rightMotor.setInverted(rightInverted);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//    rightMotor.getEncoder().setVelocityConversionFactor(1 / 60d);
    rightMotor.setOpenLoopRampRate(0.25);

    // Optimistic - we need to zero if the robot has been off...
    setSoftLimits();
    enableSoftLimits();
  }

  public void stop() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }

  public LRSpeeds getSpeed() {
    return new LRSpeeds(leftMotor.get(), rightMotor.get());
  }

  public void setSpeed(LRSpeeds lrSpeed) {
    setSpeed = true;

    // Consider whether this should only be allowed if we have homed already. That may be a bit too conservative
    this.speeds = lrSpeed;

    if (speeds.left() != 0) leftHome = false;
    if (speeds.right() != 0) rightHome = false;
  }

  // Reset flags related to home sequence
  public void zero() {
    leftMotor.getEncoder().setPosition(0.0);
    rightMotor.getEncoder().setPosition(0.0);

    leftHome = true;
    rightHome = true;
    goingHome = false;
    hasBeenHomed = true;
  }

  @Override
  public void periodic() {
    if (goingHome) {
      if (leftCurrent.update() >= homeCurrentLimit) {
        leftHome = true;
      }
      if (rightCurrent.update() >= homeCurrentLimit) {
        rightHome = true;
      }
    }

    if (setSpeed) {
      // This assumes nothing is moving these components after the home sequence
      leftMotor.set(leftHome ? 0 : applyCushion(speeds.left(), leftPosition()));
      rightMotor.set(rightHome ? 0 : applyCushion(speeds.right(), rightPosition()));
    }
  }

  // Automatically ramp down if we are getting close to the soft limits
  // but bottom out at the cushionFloor so we don't stall.
  // further. preserve speeds that are already within these bounds as-is.
  // TODO - we should build this into the motor control itself.
  double applyCushion(double speed, double position) {
    if (!hasBeenHomed) return speed;
    double delta;  // How close are we?
    double limit;

    if (abs(position - forwardSoftLimit) < cushion) {
       delta = abs(position - forwardSoftLimit);
    } else if (abs(position - reverseSoftLimit) < cushion) {
       delta = abs(position - reverseSoftLimit);
    } else return speed;

    limit = cushionFloor + (delta / cushion) * (1 - cushionFloor);

    if (abs(speed) < limit) return speed;
    return copySign(limit, speed);
  }

  public void goHome() {
    setSpeed = true;
    goingHome = true;

    // Don't call setSpeed directly. That will mess up the home sequence.
    speeds = new LRSpeeds(homeSpeed, homeSpeed);
  }

  public boolean isHome() {
    if (leftHome && rightHome) {
      hasBeenHomed = true;
      System.out.println("Climber is home");
    }
    return leftHome && rightHome;
  }

  public abstract void disableAllLimits();

  public abstract void enableAllLimits();

  abstract void setSoftLimits();

  public void disableSoftLimits() {
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    System.out.println(getName() + ".disableSoft()");
  }

  public void enableSoftLimits() {
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    System.out.println(getName() + ".enableLimits()");
  }

  protected void setSoftLimits(double forward, double reverse) {
    forwardSoftLimit = -forward * rotationsPerUnitLength;
    reverseSoftLimit = -reverse * rotationsPerUnitLength;
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forwardSoftLimit);
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forwardSoftLimit);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    System.out.println(getName() + ".setSoftLimits(); forward: " + forwardSoftLimit + ", reverse: " + reverseSoftLimit);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("left position", this::leftPosition, null);
    builder.addDoubleProperty("right position", this::rightPosition, null);
  }

  @Log(name = "left position")
  public double leftPosition() {
    return -leftMotor.getEncoder().getPosition() / rotationsPerUnitLength;
  }

  @Log(name = "right position")
  public double rightPosition() {
    return -rightMotor.getEncoder().getPosition() / rotationsPerUnitLength;
  }

  public void leftPID(State state) {
    setSpeed = false;
    this.pidOutput = leftPIDController.calculate(leftPosition(), state.position);
    //this.feedForwardOutputs = clamp(feedForward(state.velocity), -12, 12);
    this.feedForwardOutputs = feedForward(state.velocity);
    leftMotor.setVoltage(-clamp((pidOutput + feedForwardOutputs),-kNeo550NominalVoltage,kNeo550NominalVoltage));
  }

  public void rightPID(State state) {
    setSpeed = false;
    double pid = rightPIDController.calculate(rightPosition(), state.position);
    double feed = feedForward(state.velocity);
    rightMotor.setVoltage(-clamp((pid + feed),-12,12));
  }

  public abstract double feedForward(double velocity);


}
