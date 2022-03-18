package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

    public enum Height{
        LOW(kShooterLowRPS),
        HIGH(kShooterHighRPS);

        private final double rps;

        Height(double rps) {
            this.rps = rps;
        }
    }

    private final StormSpark motor = new StormSpark(kShooterId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxPIDController pidController = motor.getPIDController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kShooterS, kShooterV, kShooterA);
  private final ExponentialAverage averageSpeed = new ExponentialAverage(this::getSpeed, 8);
    public Height mode = Height.LOW;

    public Shooter() {
        motor.setInverted(false);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.getEncoder().setVelocityConversionFactor(1 / 60d); //from rpm to rps
        Shuffleboard.getTab("Shoot Command").add(this);
    }


    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

  public double getExponentialSpeed() {
    return averageSpeed.update();
  }

    public void setSpeed(double speed) {
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public void off() {
        motor.set(0);
    }

    public double setpoint() {
        return mode.rps;
    }

    public void runToSpeed(double pidOutput){
        motor.setVoltage(pidOutput + feedforward.calculate(setpoint(), 0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("speed", motor.getEncoder()::getVelocity, null);
    }
}
