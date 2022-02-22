package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

public class Shooter extends SubsystemBase {

    private final StormSpark motor = new StormSpark(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private double speed = 0;

    public Shooter() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        Shuffleboard.getTab("Shooter").addBoolean("ready2shoot", this::isReady);
        Shuffleboard.getTab("Shooter").addNumber("Speed", this::getSpeed);
    }

    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

    public void setSpeed(double speed) {
        this.speed = Math.copySign(Math.min(Math.abs(speed), 1), speed);
    }

    @Override
    public void periodic() {
        motor.set(speed);
    }

    public boolean isReady() {
        return getSpeed() > 2200;
    }

    public void on() {
        setSpeed(0.4);
    }

    public void off() {
        setSpeed(0);
    }

}
