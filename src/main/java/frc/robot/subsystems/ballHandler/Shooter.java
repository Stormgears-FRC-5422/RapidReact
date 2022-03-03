package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;


public class Shooter extends SubsystemBase {

    public enum Height{
        LOW(kShooterLowRPS),
        HIGH(kShooterHighRPS);

        public double rps;

        Height(double rps) {
            this.rps = rps;
        }
    }

    private final StormSpark motor = new StormSpark(kShooterId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxPIDController pidController = motor.getPIDController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kShooterS, kShooterV, kShooterA);
    public Height mode = Height.LOW;

    public Shooter() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.getEncoder().setVelocityConversionFactor(1 / 60d); //from rpm to rps
//        setupPID();
    }

//    private void setupPID() {
//        pidController.setP(kShooterP);
//        pidController.setI(kShooterI);
//        pidController.setD(kShooterD);
//        pidController.setFF(kShooterF);
//        pidController.setOutputRange(-1, 1);
//    }

    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

    public void setSpeed(double speed) {
        if (speed != 0) pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        else motor.set(0);
    }

    public void off() {
        setSpeed(0);
    }

    public double setpoint() {
        return mode.rps;
    }

    public void runToSpeed(double pidOutput){
//        if (pidOutput < 0) {
//            motor.setVoltage(0);
//            return;
//        }
        motor.setVoltage(pidOutput + feedforward.calculate(setpoint(), 0));
    }
}
