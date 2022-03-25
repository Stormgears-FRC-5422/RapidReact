package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.kIntakeId;
import static frc.robot.Constants.kIntakeSpeed;

public class Intake extends SubsystemBase {

  private final StormSpark motor =
      new StormSpark(kIntakeId, CANSparkMaxLowLevel.MotorType.kBrushless);

  public Intake() {
    motor.setInverted(true);
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void on() {
    motor.set(kIntakeSpeed);
  }

  public void off() {
    motor.set(0);
  }

  public void reverse() {
    motor.set(-kIntakeSpeed);
  }
}