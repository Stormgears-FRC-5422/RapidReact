package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRSpeeds;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final StormSpark leftClimber = new StormSpark(kClimberLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightClimber = new StormSpark(kClimberRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRSpeeds speeds;

    private boolean setSpeed = true;

    public Climber() {
        System.out.println("Climber()");
        speeds = new LRSpeeds();

        leftClimber.setInverted(kClimberLeftInverted);
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftClimber.getEncoder().setVelocityConversionFactor(1/60d);

        rightClimber.setInverted(kClimberRightInverted);
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimber.getEncoder().setVelocityConversionFactor(1/60d);

        // Optimistic - we need to zero if the robot has been off...
        enableLimits();
        Shuffleboard.getTab("Climber").add(this);
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

    public LRSpeeds getSpeed() {
        return new LRSpeeds(leftClimber.get(), rightClimber.get());
    }

    public void setSpeed(LRSpeeds lrSpeed) {
        setSpeed = true;
        this.speeds = lrSpeed;
    }

    public void setGoal(double left, double right) {
        setSpeed = false;
        leftClimber.getPIDController().setReference(-left, CANSparkMax.ControlType.kVelocity);
        rightClimber.getPIDController().setReference(-right, CANSparkMax.ControlType.kVelocity);
    }

    public void zero() {
        leftClimber.getEncoder().setPosition(0.0);
        rightClimber.getEncoder().setPosition(0.0);

        setLimits(-20.0f, -200.0f,-20.0f, -200.0f);
        enableLimits();

        System.out.println("Climber.zero()");
    }

    public void disableLimits() {
        leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        System.out.println("Climber.disableLimits()");
    }

    public void enableLimits() {
        leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightClimber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        System.out.println("Climber.enableLimits()");
    }

    public void setLimits(float forwardLeft, float reverseLeft, float forwardRight, float reverseRight) {
        leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardLeft);
        leftClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseLeft);
        rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, forwardRight);
        rightClimber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, reverseRight);
        System.out.println("Climber.setLimits()");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("left velocity", leftClimber.getEncoder()::getVelocity, null);
        builder.addDoubleProperty("right velocity", rightClimber.getEncoder()::getVelocity, null);
    }

    public double leftPosition() { return leftClimber.getEncoder().getPosition(); }
    public double rightPosition() { return rightClimber.getEncoder().getPosition(); }
}

