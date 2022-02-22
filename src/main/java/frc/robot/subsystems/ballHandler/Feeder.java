package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.LimitSwitch;
import frc.utils.motorcontrol.StormSpark;

public class Feeder extends SubsystemBase {

    private final StormSpark motor = new StormSpark(Constants.FEEDER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final LimitSwitch limitSwitch = new LimitSwitch(0, true);
    private double speed = 0;

    public Feeder() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        Shuffleboard.getTab("Shooter").addNumber("FeederSpeed", this::getSpeed);
        Shuffleboard.getTab("Shooter").addBoolean("tripped", this::getLimit);
        Shuffleboard.getTab("Shooter").addBoolean("enabled", limitSwitch::isEnabled);

    }

    @Override
    public void periodic() {
        motor.set(speed);
    }

    public double getSpeed() {
        return speed;
    }

    private void setSpeed(double speed) {
        if (getLimit()) this.speed = 0;
        else this.speed = Math.copySign(Math.min(Math.abs(speed), 1), speed);
    }

    public boolean getLimit() {
        return limitSwitch.get();
    }

    public void setLimit(boolean limit) {
        if (limit) limitSwitch.enable();
        else limitSwitch.disable();
    }

    public void off() {
        setSpeed(0);
    }

    public void on() {
        setSpeed(0.25);
    }
}
