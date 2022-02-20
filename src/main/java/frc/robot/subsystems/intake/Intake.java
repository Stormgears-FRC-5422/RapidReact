package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

public class Intake extends SubsystemBase {
    public enum TestMode {
        intake, feeder, shooter;
    }

    StormSpark intakeMotor;
    StormSpark feederMotor;
    StormSpark shooterMotor;

    StormSpark activeMotor;

    private TestMode mode = TestMode.intake;
    public Intake() {
        System.out.println("Creating Intake Subsystem");

        if (Constants.useIntake) {
            intakeMotor = new StormSpark(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            intakeMotor.setInverted(true);

        }

        if (Constants.useFeeder) {
            feederMotor = new StormSpark(Constants.FEEDER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            feederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        if (Constants.useShooter) {
            shooterMotor = new StormSpark(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        setModeIntake();
    }

    public void setModeIntake() {
        mode = TestMode.intake;
        activeMotor = intakeMotor;
        System.out.println("Intake Mode!");
    }

    public void setModeFeeder() {
        mode = TestMode.feeder;
        activeMotor = feederMotor;
        System.out.println("Feeder Mode!");
    }

    public void setModeShooter() {
        mode = TestMode.shooter;
        activeMotor = shooterMotor;
        System.out.println("Shooter Mode!");
    }

    public void set(double speed) {
        activeMotor.set(speed);
    }
}
