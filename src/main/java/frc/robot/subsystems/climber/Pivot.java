package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRPair;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Pivot extends SubsystemBase {
    private final StormSpark leftPivot = new StormSpark(kPivotLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightPivot = new StormSpark(kPivotRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRPair lrSetSpeed;

    public Pivot() {
        lrSetSpeed = new LRPair();

        leftPivot.setInverted(kPivotLeftInverted);
        leftPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightPivot.setInverted(kPivotRightInverted);
        rightPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        leftPivot.set(lrSetSpeed.left);
        rightPivot.set(lrSetSpeed.right);

        SmartDashboard.putNumber("pivot left current", leftPivot.getOutputCurrent());
        SmartDashboard.putNumber("pivot right current", rightPivot.getOutputCurrent());
        SmartDashboard.putNumber("pivot left position", leftPivot.getEncoder().getPosition());
        SmartDashboard.putNumber("pivot right position", rightPivot.getEncoder().getPosition());

    }

    public void stop() {
        setSpeed(new LRPair(0,0));
    }

    public LRPair getSpeed() {
        return new LRPair(leftPivot.get(),rightPivot.get());
    }

    public void setSpeed(LRPair lrSpeed) {
        this.lrSetSpeed = lrSpeed;
    }

    public void zero() {
        leftPivot.getEncoder().setPosition(0.0);
        rightPivot.getEncoder().setPosition(0.0);
        System.out.println("Pivot zero()");
    }

}

