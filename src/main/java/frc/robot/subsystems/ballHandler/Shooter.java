package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;


public class Shooter extends SubsystemBase {

    public double output;
    public double pidOutput;

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

    private double adjust = 0;

    public Shooter() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.getEncoder().setVelocityConversionFactor(1 / 60d); //from rpm to rps
    }


    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

    public void setSpeed(double speed) {
        if (speed != 0) pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        else motor.set(0);
    }

    public void off() {
        setSpeed(0);
        output = 0;
        pidOutput = 0;
    }

    public double setpoint() {
        return mode.rps;
    }

    public void runToSpeed(double pidOutput){
        this.pidOutput = pidOutput;
        output = pidOutput + feedforward.calculate(setpoint(), 0);
        motor.setVoltage(output);
    }

    public void setAdjust(double adjust) {
        this.adjust = adjust;
    }

    public double getAdjust(){
        return adjust;
    }
}
