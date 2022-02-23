package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

public class Intake extends SubsystemBase {

    private final StormSpark motor = new StormSpark(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private double speed = 0;

    public Intake() {
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = Math.copySign(Math.min(Math.abs(speed), 1), speed);
    }

    @Override
    public void periodic() {
        super.periodic();
        motor.set(speed);
    }

    public void on() {
        setSpeed(Constants.intakeSpeed);
    }

    public void off() {
        setSpeed(0);
    }
}
