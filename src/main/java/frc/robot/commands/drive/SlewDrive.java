package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.kXPrecision;
import static frc.robot.Constants.kZPrecision;

public class SlewDrive extends CommandBase {
    private final StormDrive drive;
    private final StormXboxController joystick;
    private final DifferentialDrive differentialDrive;

    private double prevSlewRate;
    private double prevTurnSlewRate;

    protected SlewRateLimiter limiter;
    protected SlewRateLimiter turnLimiter;

    public SlewDrive(StormDrive drive, StormXboxController joystick) {
        System.out.println("Just created the SlewDrive command");
        addRequirements(drive);

        this.drive = drive;
        this.joystick = joystick;
        differentialDrive = drive.getDifferentialDrive();

        prevSlewRate = drive.getSlewRate();
        prevTurnSlewRate = drive.getTurnSlewRate();
        System.out.println("Initial slewRate: " + prevSlewRate + "  initial turnSlewRate: " + prevTurnSlewRate);

        limiter = new SlewRateLimiter(prevSlewRate);
        turnLimiter = new SlewRateLimiter(prevTurnSlewRate);
    }

    @Override
    public void execute() {
        double targetSpeed = (drive.getPrecision() ? kXPrecision : 1 ) * joystick.getTriggerSpeed();
        double targetZRotation = (drive.getPrecision() ? kZPrecision : 1 ) * joystick.getLeftJoystickX();

        if (drive.getSlewRate() != prevSlewRate) {
            System.out.println("updated slewRate: " + prevSlewRate);
            prevSlewRate = drive.getSlewRate();
            limiter = new SlewRateLimiter(prevSlewRate);
        }

        if (drive.getTurnSlewRate() != prevTurnSlewRate) {
            System.out.println("updated turnSlewRate: " + prevSlewRate);
            prevTurnSlewRate = drive.getTurnSlewRate();
            turnLimiter = new SlewRateLimiter(prevTurnSlewRate);
        }

        //differentialDrive.arcadeDrive(limiter.calculate(targetSpeed), turnLimiter.calculate(targetZRotation));
        differentialDrive.curvatureDrive(limiter.calculate(targetSpeed), turnLimiter.calculate(targetZRotation),true);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}