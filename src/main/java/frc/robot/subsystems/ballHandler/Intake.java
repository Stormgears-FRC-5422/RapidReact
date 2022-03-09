package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;
import static java.lang.Math.*;

public class Intake extends SubsystemBase {

    private final StormSpark motor = new StormSpark(kIntakeId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private double speed = 0;

    public Intake() {
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = copySign(min(abs(speed), 1), speed);
    }

    @Override
    public void periodic() {
        super.periodic();
        motor.set(speed);
    }

    public void on() {
        setSpeed(kIntakeSpeed);
    }

    public void off() {
        setSpeed(0);
    }
}
