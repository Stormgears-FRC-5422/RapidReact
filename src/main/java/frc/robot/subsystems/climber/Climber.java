package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRPair;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final StormSpark leftClimber = new StormSpark(kClimberLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightClimber = new StormSpark(kClimberRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRPair lrSetSpeed;

    private boolean goingHome = false;
    private boolean leftHome = false;
    private boolean rightHome = false;
    private boolean hasBeenHomed = false;

    private final ExponentialAverage leftCurrent;
    private final ExponentialAverage rightCurrent;

    public Climber() {
        System.out.println("Climber()");
        lrSetSpeed = new LRPair();

        leftClimber.setInverted(kClimberLeftInverted);
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftClimber.setOpenLoopRampRate(0.25);

        rightClimber.setInverted(kClimberRightInverted);
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimber.setOpenLoopRampRate(0.25);

        leftCurrent = new ExponentialAverage(leftClimber::getOutputCurrent, 2);
        rightCurrent = new ExponentialAverage(rightClimber::getOutputCurrent, 2);

        // Optimistic - we need to zero if the robot has been off...
        enableLimits();
    }

    @Override
    public void periodic() {
        double lC = leftCurrent.update();
        double rC = rightCurrent.update();

        if (goingHome) {
            if (lC >= kClimberHomeCurrentLimit) {
                System.out.println("Left climber Home");
                leftHome = true;
            }
            if (rC >= kClimberHomeCurrentLimit) {
                System.out.println("Right climber Home");
                rightHome = true;
            }
        }

        // This assumes nothing is moving these components after the home sequence
        leftClimber.set(leftHome ? 0 : lrSetSpeed.left);
        rightClimber.set(rightHome ? 0 : lrSetSpeed.right);

        SmartDashboard.putNumber("climber left current", lC);
        SmartDashboard.putNumber("climber right current", rC);
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
        if (hasBeenHomed) {
            this.lrSetSpeed = lrSpeed;
            if (lrSetSpeed.left != 0) leftHome = false;
            if (lrSetSpeed.right != 0) rightHome = false;
        } else {
            System.out.println("Don't move the climbers without homing first");
        }
    }

    public void zero() {
        leftClimber.getEncoder().setPosition(0.0);
        rightClimber.getEncoder().setPosition(0.0);

        setLimits(-20.0f, -200.0f,-20.0f, -200.0f);
        enableLimits();

        // Reset flags related to home sequence
        leftHome = false;
        rightHome = false;
        goingHome = false;

        System.out.println("Climber.zero()");
    }


    public void goHome() {
        goingHome = true;

        // Don't call setSpeed directly. That will mess up the home sequence.
        lrSetSpeed = new LRPair(kClimberHomeSetSpeed, kClimberHomeSetSpeed);
    }

    public boolean isHome() {
        if (leftHome && rightHome) {
            hasBeenHomed = true;
        }

        return leftHome && rightHome;
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

