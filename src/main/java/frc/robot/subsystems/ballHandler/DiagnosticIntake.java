package frc.robot.subsystems.ballHandler;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class DiagnosticIntake extends SubsystemBase {
    StormSpark intakeMotor;
    StormSpark feederMotor;
    StormSpark shooterMotor;
    StormSpark activeMotor;

    private TestMode mode = TestMode.intake;

    public DiagnosticIntake() {

        if (kUseIntake) {
            intakeMotor = new StormSpark(kIntakeId, CANSparkMaxLowLevel.MotorType.kBrushless);
            intakeMotor.setInverted(true);
        }

        if (kUseFeeder) {
            feederMotor = new StormSpark(kFeederId, CANSparkMaxLowLevel.MotorType.kBrushless);
            feederMotor.setInverted(false);
        }

        if (kUseShooter) {
            shooterMotor = new StormSpark(kShooterId, CANSparkMaxLowLevel.MotorType.kBrushless);
            shooterMotor.setInverted(false);
        }

        setModeIntake();
    }

    public void set(double speed) {
        if (mode == TestMode.shooter) {
            speed *= 0.3;
            SmartDashboard.putNumber("shooter Speed", speed);
            SmartDashboard.putNumber("Shooter Velocity", activeMotor.getEncoder().getVelocity());
        }
        if (mode == TestMode.intake) {
            speed *= 0.5;
            SmartDashboard.putNumber("intake speed", speed);
        }
        if (mode == TestMode.feeder) {
            speed *= 0.25;
        }
        if (activeMotor != null ) activeMotor.set(speed);
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
