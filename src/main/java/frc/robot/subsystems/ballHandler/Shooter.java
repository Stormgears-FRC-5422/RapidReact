package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

public class Shooter extends SubsystemBase {

    private final StormSpark motor = new StormSpark(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxPIDController pidController = motor.getPIDController();

    public Shooter() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.getEncoder().setVelocityConversionFactor(1 / 60d); //from rpm to rps
//        pidController.setP(0.005);
        pidController.setP(0);
        pidController.setI(0);
        pidController.setD(0);
//        pidController.setD(-.02);
        pidController.setOutputRange(-1, 1);
        pidController.setFF(0.011);
        Shuffleboard.getTab("Shooter").addBoolean("ready2shoot", this::isReady);
        Shuffleboard.getTab("Shooter").addNumber("Speed", this::getSpeed);
    }

    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

    public void setSpeed(double speed) {
        if (speed != 0) pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        else motor.set(0);
    }

    public boolean isReady() {
        return getSpeed() >= 0.95 * Constants.shooterLowRPM;
    }

    public void off() {
        setSpeed(0);
    }
}
