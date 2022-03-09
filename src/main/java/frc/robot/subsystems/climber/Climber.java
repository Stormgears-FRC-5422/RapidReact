package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
    }

    @Override
    public void periodic() {
        leftClimber.set(lrSetSpeed.left);
        rightClimber.set(lrSetSpeed.right);
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

    public double getCurrent() {
        return Math.max(leftClimber.getOutputCurrent(), rightClimber.getOutputCurrent());
    }

}

