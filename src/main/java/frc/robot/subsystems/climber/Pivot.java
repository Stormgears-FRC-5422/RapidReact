package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRPair;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Pivot extends SubsystemBase {
    private final StormSpark leftPivot = new StormSpark(kPivotLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightPivot = new StormSpark(kPivotRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRPair lrSetSpeed;
    private boolean goingHome = false;
    private boolean leftHome = false;
    private boolean rightHome = false;
    private boolean hasBeenHomed = false;

    private final ExponentialAverage leftCurrent;
    private final ExponentialAverage rightCurrent;

    public Pivot() {
        lrSetSpeed = new LRPair();

        leftPivot.setInverted(kPivotLeftInverted);
        leftPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftPivot.setOpenLoopRampRate(0.25);

        rightPivot.setInverted(kPivotRightInverted);
        rightPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightPivot.setOpenLoopRampRate(0.25);

        leftCurrent = new ExponentialAverage(leftPivot::getOutputCurrent, 2);
        rightCurrent = new ExponentialAverage(rightPivot::getOutputCurrent, 2);

        // Optimistic - we need to zero if the robot has been off...
        enableLimits();
    }

    @Override
    public void periodic() {
        double lC = leftCurrent.update();
        double rC = rightCurrent.update();

        if (goingHome) {
            if (lC >= kPivotHomeCurrentLimit) {
                System.out.println("Left Pivot Home");
                leftHome = true;
            }
            if (rC >= kPivotHomeCurrentLimit) {
                System.out.println("Right Pivot Home");
                rightHome = true;
            }
        }

        // This assumes nothing is moving these components after the home sequence
        leftPivot.set(leftHome ? 0 : lrSetSpeed.left);
        rightPivot.set(rightHome ? 0 : lrSetSpeed.right);

        SmartDashboard.putNumber("pivot left current", lC);
        SmartDashboard.putNumber("pivot right current", rC);
        SmartDashboard.putNumber("pivot left position", leftPivot.getEncoder().getPosition());
        SmartDashboard.putNumber("pivot right position", rightPivot.getEncoder().getPosition());
    }

    public void stop() {
        setSpeed(new LRPair(0, 0));
    }

    public LRPair getSpeed() {
        return new LRPair(leftPivot.get(), rightPivot.get());
    }

    public void setSpeed(LRPair lrSpeed) {
        if (hasBeenHomed) {
            this.lrSetSpeed = lrSpeed;
            if (lrSetSpeed.left != 0) leftHome = false;
            if (lrSetSpeed.right != 0) rightHome = false;
        } else {
            System.out.println("Don't move the pivot without homing first");
        }

    }

    public void zero() {
        leftPivot.getEncoder().setPosition(0.0);
        rightPivot.getEncoder().setPosition(0.0);

        setLimits(-20.0f, -200.0f, -20.0f, -200.0f);
        enableLimits();

        // Reset flags related to home sequence
        leftHome = false;
        rightHome = false;
        goingHome = false;

        System.out.println("Pivot.zero()");
    }

    public void goHome() {
        goingHome = true;

        // Don't call setSpeed directly. That will mess up the home sequence.
        lrSetSpeed = new LRPair(kPivotHomeSetSpeed, kPivotHomeSetSpeed);
    }

    public boolean isHome() {
        if (leftHome && rightHome) {
            hasBeenHomed = true;
        }

        return leftHome && rightHome;
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

}

