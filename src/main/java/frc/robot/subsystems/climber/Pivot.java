package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRSpeeds;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Pivot extends SubsystemBase {
    private final StormSpark leftPivot = new StormSpark(kPivotLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightPivot = new StormSpark(kPivotRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRSpeeds speeds;

    public Pivot() {
        speeds = new LRSpeeds();

        leftPivot.setInverted(kPivotLeftInverted);
        leftPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightPivot.setInverted(kPivotRightInverted);
        rightPivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        leftPivot.set(speeds.left());
        rightPivot.set(speeds.right());

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
        System.out.println("Pivot zero()");
    }

}

