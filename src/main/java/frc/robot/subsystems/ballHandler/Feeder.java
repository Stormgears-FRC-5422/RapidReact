package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.LimitSwitch;
import frc.utils.motorcontrol.StormSpark;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.*;
import static java.lang.Math.*;

public class Feeder extends SubsystemBase implements Loggable {
  //  private static final String shuffleboardTabName = "Shooter";

  private final StormSpark motor =
      new StormSpark(
          kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);
  private final LimitSwitch limitSwitch = new LimitSwitch(0, true);
  private double speed = 0;
  private double liftVoltage = 0;
  private boolean forward = true;

  public Feeder() {
    motor.setInverted(false);
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor.getEncoder().setPosition(0.0);
  }

  @Override
  public void periodic() {
    if (forward) motor.set(speed);
    else motor.setVoltage(liftVoltage);
  }

  @Log(name = "feederSpeed")
  public double getSpeed() {
    return speed;
  }

  private void setSpeed(double speed) {
    if (limitSwitch.getAbsoluteLimit()) motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    if (getLimit()) this.speed = 0;
    else this.speed = copySign(min(abs(speed), 1), speed);
  }

  public double getMotorPosition() {
    return motor.getEncoder().getPosition();
  }

  @Log.BooleanBox(name = "limitSwitch")
  public boolean getLimit() {
    return limitSwitch.get();
  }

  public void setLimit(boolean limit) {
    if (limit) limitSwitch.enable();
    else limitSwitch.disable();
  }

  public boolean getAbsoluteLimit() {
    return limitSwitch.getAbsoluteLimit();
  }

  public void off() {
    setSpeed(0);
    forward = true;
  }

  public void intakeOn() {
    forward = true;
    setSpeed(kFeederIntakeSpeed);
  }

  public void shootOn() {
    forward = true;
    setSpeed(kFeederShootingSpeed);
  }

  public void reverse() {
    forward = false;
    motor.set(-kFeederIntakeSpeed);
  }

  public void initReverse() {
    // There are some issues here if we try to lift this while the motor is already spinning.
    // really we should wait for it to stop.
    forward = false;
    speed = 0;
    motor.set(0);
    //
    //    motor.getEncoder().setPosition(14); // this is fully down
    //    motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    //    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1.75f);
    //    System.out.println("Set motor position to " + getMotorPosition());
  }

  public double getLiftVoltage() {
    return liftVoltage;
  }

  public void setLift(double v) {
    double targetPos = 1.75; // unscaled rotations from all the way back to straight up.
    double extent = 14 - targetPos; // unscaled rotations from all the way back to straight up.
    double currentPos = motor.getEncoder().getPosition();
    double scale = sin((PI / 2) * ((currentPos - targetPos) / extent));

    double maxVoltage = 12;
    liftVoltage = clamp(-scale * v * maxVoltage, -maxVoltage, 0);

    System.out.println("setLift " + liftVoltage);
  }

  public void setCoast() {
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
}
