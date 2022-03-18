package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber.HoldTargetPosition;
import frc.utils.LRSpeeds;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;
// TODO add soft limits to constants
public abstract class ClimberParentSystem extends SubsystemBase {

  protected final String shuffleBoardTabName = this.getName();

  protected final StormSpark leftMotor;
  protected final StormSpark rightMotor;

  protected final ExponentialAverage leftCurrent;
  protected final ExponentialAverage rightCurrent;

  protected final PIDController leftPIDController;
  protected final PIDController rightPIDController;

  protected final double homeCurrentLimit;
  protected final double homeSpeed;
  private final HoldTargetPosition holdTargetPosition;

  protected LRSpeeds speeds = new LRSpeeds();

  protected boolean setSpeed = true;
  protected boolean goingHome = false;
  protected boolean leftHome = false;
  protected boolean rightHome = false;
  protected boolean hasBeenHomed = false;

  // For monitoring on Shuffleboard, does nothing
  protected double pidOutput = 0;
  protected double feedForwardOutputs = 0;

  protected ClimberParentSystem(
      int leftMotorID,
      int rightMotorID,
      boolean leftInverted,
      boolean rightInverted,
      PIDController leftPIDController,
      PIDController rightPIDController,
      double homeCurrentLimit,
      double homeSpeed) {
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
    enableLimits();
    shuffleBoard();
  }

  private void shuffleBoard() {
    Shuffleboard.getTab(shuffleBoardTabName).add(this);
    Shuffleboard.getTab(shuffleBoardTabName).add("leftPID", this.leftPIDController);
    Shuffleboard.getTab(shuffleBoardTabName).add("rightPID", this.rightPIDController);
    Shuffleboard.getTab(shuffleBoardTabName).addNumber("PIDOutput", () -> pidOutput);
    Shuffleboard.getTab(shuffleBoardTabName).addNumber("FFOutput", () -> feedForwardOutputs);
    Shuffleboard.getTab(shuffleBoardTabName)
        .addNumber("CombinedOutput", () -> pidOutput + feedForwardOutputs);
    Shuffleboard.getTab(shuffleBoardTabName).addBoolean("setSpeed", () -> setSpeed);
    Shuffleboard.getTab(shuffleBoardTabName).addNumber("lC", leftCurrent::update);
    Shuffleboard.getTab(shuffleBoardTabName).addNumber("rC", rightCurrent::update);
    Shuffleboard.getTab(shuffleBoardTabName)
        .addString(
            "command running",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
    }

  public void stop() {
    setSpeed(LRSpeeds.stop());
  }

  public LRSpeeds getSpeed() {
    return new LRSpeeds(leftMotor.get(), rightMotor.get());
  }

  public void setSpeed(LRSpeeds lrSpeed) {
    setSpeed = true;

    if (hasBeenHomed) {
      this.speeds = lrSpeed;
    }

    if (speeds.left() != 0) leftHome = false;
    if (speeds.right() != 0) rightHome = false;
  }

  // Reset flags related to home sequence
  public void zero() {
    leftMotor.getEncoder().setPosition(0.0);
    rightMotor.getEncoder().setPosition(0.0);

    setLimits();
    enableLimits();

    leftHome = false;
    rightHome = false;
    goingHome = false;
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
      leftMotor.set(leftHome ? 0 : speeds.left());
      rightMotor.set(rightHome ? 0 : speeds.right());
    }
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

  public void disableLimits() {
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    System.out.println("Climber.disableLimits()");
  }

  public void enableLimits() {
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    System.out.println("Climber.enableLimits()");
  }

  void setLimits(double forward, double reverse) {
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forward);
    leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverse);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forward);
    rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverse);
    System.out.println("Climber.setLimits()");
  }

  abstract void setLimits();

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("left position", this::leftPosition, null);
    builder.addDoubleProperty("right position", this::rightPosition, null);
  }

  public double leftPosition() {
    return -leftMotor.getEncoder().getPosition();
  }

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
    this.getCurrentCommand().end(true);
    holdTargetPosition.updateTargetPosition(target);
    holdTargetPosition.schedule();
  }
}
