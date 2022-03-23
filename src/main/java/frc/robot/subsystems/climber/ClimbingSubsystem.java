package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber.hold.HoldTargetPosition;
import frc.utils.LRSpeeds;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static java.lang.Math.*;

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

  protected final double homeCurrentLimit;
  protected final double homeSpeed;
  protected final double cushion;
  protected final double cushionFloor;

  @Log.Exclude private final HoldTargetPosition holdTargetPosition;
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

  protected ClimbingSubsystem(
      int leftMotorID,
      int rightMotorID,
      boolean leftInverted,
      boolean rightInverted,
      PIDController leftPIDController,
      PIDController rightPIDController,
      double homeCurrentLimit,
      double homeSpeed,
      double cushion,
      double cushionFloor) {
    leftMotor = new StormSpark(leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor = new StormSpark(rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    this.leftCurrent = new ExponentialAverage(leftMotor::getOutputCurrent, 4);
    this.rightCurrent = new ExponentialAverage(rightMotor::getOutputCurrent, 4);

    this.leftPIDController = leftPIDController;
    this.rightPIDController = rightPIDController;

    //    leftPIDController.setTolerance(4);
    //    rightPIDController.setTolerance(4);

    this.homeCurrentLimit = homeCurrentLimit;
    this.homeSpeed = homeSpeed;
    this.cushion = cushion;
    this.cushionFloor = cushionFloor;

    this.holdTargetPosition = new HoldTargetPosition(this, this.leftPosition());

    leftMotor.setInverted(leftInverted);
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotor.getEncoder().setVelocityConversionFactor(1 / 60d);
    leftMotor.setOpenLoopRampRate(0.25);

    rightMotor.setInverted(rightInverted);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.getEncoder().setVelocityConversionFactor(1 / 60d);
    rightMotor.setOpenLoopRampRate(0.25);

    // Optimistic - we need to zero if the robot has been off...
    setSoftLimits();
    enableSoftLimits();
    allLimitsOn = false;
    shuffleBoard();
  }

  private void shuffleBoard() {
    //    Shuffleboard.getTab(shuffleBoardTabName).add(this);
    //    Shuffleboard.getTab(shuffleBoardTabName).add("leftPID", this.leftPIDController);
    //    Shuffleboard.getTab(shuffleBoardTabName).add("rightPID", this.rightPIDController);
    //    Shuffleboard.getTab(shuffleBoardTabName).addNumber("PIDOutput", () -> pidOutput);
    //    Shuffleboard.getTab(shuffleBoardTabName).addNumber("FFOutput", () -> feedForwardOutputs);
    //    Shuffleboard.getTab(shuffleBoardTabName)
    //        .addNumber("CombinedOutput", () -> pidOutput + feedForwardOutputs);
    //    Shuffleboard.getTab(shuffleBoardTabName).addBoolean("setSpeed", () -> setSpeed);
    //    Shuffleboard.getTab(shuffleBoardTabName).addNumber("lC", leftCurrent::update);
    //    Shuffleboard.getTab(shuffleBoardTabName).addNumber("rC", rightCurrent::update);
    //    Shuffleboard.getTab(shuffleBoardTabName)
    //        .addString(
    //            "command running",
    //            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
  }

  public void stop() {
    setSpeed(LRSpeeds.stop());
  }

  public LRSpeeds getSpeed() {
    return new LRSpeeds(leftMotor.get(), rightMotor.get());
  }

  public void setSpeed(LRSpeeds lrSpeed) {
    setSpeed = true;

    //    if (hasBeenHomed) {
    this.speeds = lrSpeed;
    //    }

    if (speeds.left() != 0) leftHome = false;
    if (speeds.right() != 0) rightHome = false;
  }

  // Reset flags related to home sequence
  public void zero() {
    leftMotor.getEncoder().setPosition(0.0);
    rightMotor.getEncoder().setPosition(0.0);

    leftHome = false;
    rightHome = false;
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
      leftMotor.set(leftHome ? 0 : applyCushion(speeds.left(),leftPosition()));
      rightMotor.set(rightHome ? 0 : applyCushion(speeds.right(), rightPosition()));
    }
  }

  // Automatically ramp down if we are getting close to the soft limits
  // but bottom out at the cushionFloor so we don't stall.
  // further. preserve speeds that are already within these bounds as-is.
  double applyCushion(double speed, double position) {
    if (!hasBeenHomed) return speed;
    double delta;  // How close are we?
    double limit;

    if (abs(position - forwardSoftLimit) < cushion) {
       delta = abs(position - forwardSoftLimit);
    } else if (abs(position - reverseSoftLimit) < cushion) {
       delta = abs(position - reverseSoftLimit);
    } else return speed;

    //limit = cushionFloor + (delta / cushion) * (1 - cushionFloor);
    limit = cushionFloor;

    //System.out.println("Name: " + name + " limit: " + limit + " speed: " + speed + " position: " + position + " forwardSoftLimit: " + forwardSoftLimit + " reverseSoftLimit: " + reverseSoftLimit);

    return ( abs(speed) < limit ? speed : copySign(limit, speed) );
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

  public void overrideHome() {
    hasBeenHomed = true;
  }

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
    forwardSoftLimit = forward;
    reverseSoftLimit = reverse;
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forward);
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverse);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forward);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverse);
    System.out.println("Climber.setLimits()");
  }

  abstract void setSoftLimits();

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("left position", this::leftPosition, null);
    builder.addDoubleProperty("right position", this::rightPosition, null);
  }

  @Log(name = "left position")
  public double leftPosition() {
    return -leftMotor.getEncoder().getPosition();
  }

  @Log(name = "right position")
  public double rightPosition() {
    return -rightMotor.getEncoder().getPosition();
  }

  public void leftPID(State state) {
    setSpeed = false;
    this.pidOutput =
        MathUtil.clamp(leftPIDController.calculate(leftPosition(), state.position), -12, 12);
    this.feedForwardOutputs = feedForward(state.velocity);
    leftMotor.setVoltage(-(pidOutput + feedForwardOutputs));
  }

  public void rightPID(State state) {
    setSpeed = false;
    double pid =
        MathUtil.clamp(rightPIDController.calculate(rightPosition(), state.position), -12, 12);
    double feed = feedForward(state.velocity);
    rightMotor.setVoltage(-(pid + feed));
  }

  public abstract double feedForward(double velocity);

  public void holdTarget(double target) {
    CommandScheduler.getInstance().cancelAll();
    holdTargetPosition.updateTargetPosition(target);
    holdTargetPosition.schedule();
  }

  public abstract void disableAllLimits();

  public boolean isAllLimitsEnabled() {
    return allLimitsOn;
  }

  public abstract void enableAllLimits();

}
