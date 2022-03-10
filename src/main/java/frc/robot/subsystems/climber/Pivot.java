package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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

    public double getCurrent() {
        return Math.max(leftPivot.getOutputCurrent(), rightPivot.getOutputCurrent());
    }

}

