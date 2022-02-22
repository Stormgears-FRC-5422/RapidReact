package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

public class DiagnosticIntake extends SubsystemBase {
    public DiagnosticIntake() {
        System.out.println("Creating Intake Subsystem");

        if (Constants.useIntake)
            intakeMotor = new StormSpark(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        if (Constants.useFeeder) {
            feederMotor = new StormSpark(Constants.FEEDER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            feederMotor.setInverted(false);
        }

        if (Constants.useShooter) {
            shooterMotor = new StormSpark(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            shooterMotor.setInverted(false);
        }


        setModeIntake();
    }

    StormSpark intakeMotor;
    StormSpark feederMotor;
    StormSpark shooterMotor;

    StormSpark activeMotor;

    private TestMode mode = TestMode.intake;

    public void set(double speed) {
        if (mode == TestMode.shooter) {
            speed *= 0.3;
            SmartDashboard.putNumber("shooter Speed", speed);
            SmartDashboard.putNumber("Shooter Velocity", activeMotor.getEncoder().getVelocity());
        }
        if (mode == TestMode.intake) {
            SmartDashboard.putNumber("intake speed", speed);
        }
        if (mode == TestMode.feeder) {
            speed *= 0.25;
        }
        activeMotor.set(speed);
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

    public enum TestMode {
        intake, feeder, shooter
    }
}
