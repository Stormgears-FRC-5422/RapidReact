package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.LRSpeeds;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Pivot extends ClimberParentSystem {
    protected final PIDController leftPIDController = new PIDController(0.05, 0, 0);
    protected final PIDController rightPIDController = new PIDController(0.05, 0, 0);
    private final StormSpark leftPivot = new StormSpark(kPivotLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightPivot = new StormSpark(kPivotRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double kS = 0.05;

    private LRSpeeds speeds;

    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0.0686, 0);

    public Pivot() {
        speeds = new LRSpeeds();

        leftPivot.setInverted(kPivotLeftInverted);
        leftPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftPivot.getEncoder().setVelocityConversionFactor(1/60d);

        rightPivot.setInverted(kPivotRightInverted);
        rightPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightPivot.getEncoder().setVelocityConversionFactor(1/60d);

        Shuffleboard.getTab("Pivot").add(this);
        Shuffleboard.getTab("Pivot").add("leftPID", leftPIDController);
        Shuffleboard.getTab("Pivot").add("rightPID", rightPIDController);

        Shuffleboard.getTab("Pivot").addNumber("PIDOutput", () -> pidOutput);
        Shuffleboard.getTab("Pivot").addNumber("FeedForwardOutputs", () -> feedForwardOutputs);
        Shuffleboard.getTab("Pivot").addNumber("Combined", () -> pidOutput + feedForwardOutputs);
        Shuffleboard.getTab("Pivot").addBoolean("setSpeed", () -> setSpeed);
        // Optimistic - we need to zero if the robot has been off...
        enableLimits();
    }

    @Override
    public void periodic() {
    if (setSpeed) {
      leftPivot.set(speeds.left());
      rightPivot.set(speeds.right());
        }

        SmartDashboard.putNumber("pivot left current", leftPivot.getOutputCurrent());
        SmartDashboard.putNumber("pivot right current", rightPivot.getOutputCurrent());
        SmartDashboard.putNumber("pivot left position", leftPivot.getEncoder().getPosition());
        SmartDashboard.putNumber("pivot right position", rightPivot.getEncoder().getPosition());

    }

    public void stop() {
        setSpeed(LRSpeeds.stop());
    }

    public LRSpeeds getSpeed() {
        return new LRSpeeds(leftPivot.get(), rightPivot.get());
    }

    public void setSpeed(LRSpeeds lrSpeed) {
        this.speeds = lrSpeed;
    }

    public void zero() {
        leftPivot.getEncoder().setPosition(0.0);
        rightPivot.getEncoder().setPosition(0.0);

        setLimits(-20.0f, -200.0f,-20.0f, -200.0f);
        enableLimits();

        System.out.println("Pivot.zero()");
    }

    public void disableLimits() {
        leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        System.out.println("Pivot.disableLimits()");
    }

    public void enableLimits() {
        leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        System.out.println("Pivot.enableLimit()");
    }

    public void setLimits(float forwardLeft, float reverseLeft, float forwardRight, float reverseRight) {
        leftPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardLeft);
        leftPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseLeft);
        rightPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardRight);
        rightPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseRight);
        System.out.println("Pivot.setLimits()");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("left pivot position", this::leftPosition, null);
        builder.addDoubleProperty("right pivot position", this::rightPosition, null);

    }

    @Override
    public double leftPosition() {
        return -leftPivot.getEncoder().getPosition();
    }

    @Override
    public double rightPosition() {
        return -rightPivot.getEncoder().getPosition();
    }

    @Override
    public void leftPID(TrapezoidProfile.State state) {
        setSpeed = false;
        this.pidOutput =
                MathUtil.clamp(leftPIDController.calculate(leftPosition(), state.position), -12, 12);
        this.feedForwardOutputs = feedforward.calculate(state.velocity, 0) + kS * Math.signum(state.position - leftPosition());
        leftPivot.setVoltage(-(pidOutput + feedForwardOutputs));
    }

    @Override
    public void rightPID(TrapezoidProfile.State state) {
        setSpeed = false;
        double pid =
                MathUtil.clamp(rightPIDController.calculate(rightPosition(), state.position), -12, 12);
        double feed = feedforward.calculate(state.velocity, 0) + kS * Math.signum(state.position - leftPosition());
        rightPivot.setVoltage(-(pid + feed));
    }
}

