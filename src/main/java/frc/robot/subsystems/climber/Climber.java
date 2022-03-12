package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRPair;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final StormSpark leftClimber = new StormSpark(kClimberLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightClimber = new StormSpark(kClimberRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRPair lrSetSpeed;

    public Climber() {
        System.out.println("Climber()");
        lrSetSpeed = new LRPair();

        leftClimber.setInverted(kClimberLeftInverted);
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightClimber.setInverted(kClimberRightInverted);
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Optimistic - we need to zero if the robot has been off...
        enableLimits();
    }

    @Override
    public void periodic() {
        leftClimber.set(lrSetSpeed.left);
        rightClimber.set(lrSetSpeed.right);

        SmartDashboard.putNumber("climber left current", leftClimber.getOutputCurrent());
        SmartDashboard.putNumber("climber right current", rightClimber.getOutputCurrent());
        SmartDashboard.putNumber("climber left position", leftClimber.getEncoder().getPosition());
        SmartDashboard.putNumber("climber right position", rightClimber.getEncoder().getPosition());
    }

    public void stop() {
        setSpeed(new LRPair(0,0));
    }

    public LRPair getSpeed() {
        return new LRPair(leftClimber.get(),rightClimber.get());
    }

    public void setSpeed(LRPair lrSpeed) {
        this.lrSetSpeed = lrSpeed;
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




}

