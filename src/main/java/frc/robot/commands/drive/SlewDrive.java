package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SparkDrive;
import frc.utils.joysticks.DriveJoystick;


public class SlewDrive extends CommandBase {
    private final SparkDrive drive;
    private DriveJoystick joystick;

    private SlewRateLimiter limiter;
    private SlewRateLimiter turn_limiter;

    private double m_prev_slew_rate;
    private double m_prev_turn_slew_rate;

    private double targetSpeed;

    public SlewDrive(SparkDrive drive, DriveJoystick joystick) {
        System.out.println("Just created the SlewDrive command");

        addRequirements(drive);

        this.drive = drive;
        this.joystick = joystick;

//        positiveLimiter = new SlewRateLimiter(1.5);
        limiter = new SlewRateLimiter(m_prev_slew_rate);
//        negativeLimiter = new SlewRateLimiter(2.5);
        turn_limiter = new SlewRateLimiter(m_prev_turn_slew_rate);

    }

    @Override
    public void execute() {
        targetSpeed = joystick.getXSpeed();
        SmartDashboard.putNumber("joystick speed", targetSpeed);
        drive.getDifferentialDrive().arcadeDrive(targetSpeed , joystick.getZRotation());
    }
}