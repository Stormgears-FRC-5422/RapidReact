package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRSpeeds;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final StormSpark leftClimber = new StormSpark(kClimberLeftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final StormSpark rightClimber = new StormSpark(kClimberRightId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private LRSpeeds speeds;

    public Climber() {
        System.out.println("Climber()");
        speeds = new LRSpeeds();

        leftClimber.setInverted(kClimberLeftInverted);
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightClimber.setInverted(kClimberRightInverted);
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        leftClimber.set(speeds.left());
        rightClimber.set(speeds.right());

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
        this.speeds = lrSpeed;
    }

    public void zero() {
        leftClimber.getEncoder().setPosition(0.0);
        rightClimber.getEncoder().setPosition(0.0);
        System.out.println("Climber zero()");
    }

}

