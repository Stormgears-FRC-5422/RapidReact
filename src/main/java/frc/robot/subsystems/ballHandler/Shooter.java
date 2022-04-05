package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.filters.ExponentialAverage;
import frc.utils.motorcontrol.StormSpark;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase implements Loggable {
  private final StormSpark motor =
      new StormSpark(kShooterId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(kShooterS, kShooterV, kShooterA);

  public Height mode = Height.Custom;

  private ExponentialAverage averageSpeed = new ExponentialAverage(this::getSpeed, 25);

  public Shooter() {
    motor.setInverted(false);
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor.getEncoder().setVelocityConversionFactor(1 / 60d); // from rpm to rps
    //        Shuffleboard.getTab("Shoot Command").add(this);
  }

  @Override
  public String configureLogName() {
    return "BallHandler";
  }

  @Log(name = "Speed")
  public double getSpeed() {
    return motor.getEncoder().getVelocity();
  }

  @Log(name = "Current")
  public double getCurrent() { return motor.getOutputCurrent();}
  //    @Log.Graph(name = "Speed", visibleTime = 3)

  @Log(name = "Exp Speed #")
  public double getExponentialSpeed() {
    return averageSpeed.update();
  }
  //    @Log.Graph(name = "Exp Speed", visibleTime = 3)

  public void resetExponential() {
    averageSpeed = new ExponentialAverage(this::getSpeed, 25);
  }

  public void off() {
    motor.set(0);
  }

  @Log(name = "Setpoint")
  public double setpoint() {
    return mode.rps;
  }

  @Config.NumberSlider(
      name = "New Setpoint",
      min = 10,
      max = 90,
      blockIncrement = 1,
      defaultValue = 68)
  public void setSetpoint(double setpoint) {
    mode = Height.fromRPS(setpoint);
  }

  public void runToSpeed(double pidOutput) {
    motor.setVoltage(pidOutput + feedforward.calculate(setpoint(), 0));
  }

  public enum Height {
    LOW(kShooterLowRPS),
    HIGH(kShooterHighRPS),
    Custom(kShooterHighRPS);

    private double rps;

    Height(double rps) {
      this.rps = rps;
    }

    public static Height fromRPS(double rps) {
      Custom.rps = rps;
      return Custom;
    }
  }

  //    @Override
  //    public void initSendable(SendableBuilder builder) {
  //        builder.addDoubleProperty("speed", motor.getEncoder()::getVelocity, null);
  //    }
}
