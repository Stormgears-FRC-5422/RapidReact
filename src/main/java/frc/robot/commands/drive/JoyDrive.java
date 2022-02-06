package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SafeDrive;
import frc.utils.joysticks.DriveJoystick;


public class JoyDrive extends CommandBase {
    private final SafeDrive drive;
    private DriveJoystick joystick;

    private double previousTarget;
    private double targetSpeed;
    private double currentSpeed;
    long count = 0;

    public JoyDrive(SafeDrive drive, DriveJoystick joystick) {
        addRequirements(drive);

        this.drive = drive;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        double ramp;

        // Accelerate slowly, stop quickly - in forward mode
        if (targetSpeed > 0) {
            // Going forwards
            if (previousTarget < targetSpeed) {
                ramp = 2.0;
            } else {
                ramp = 0.5;
            }
        } else {
            // Going backwards
            if (previousTarget > targetSpeed) {
                ramp = 2.0;
            } else {
                ramp = 0.8;
            }
        }

        if (++count % 50 == 0)
            System.out.println("OUTPUT: " + previousTarget + "    TARGET: " + targetSpeed + "JOY VAL: " +  joystick.getXSpeed());

        drive.driveArcade(targetSpeed,joystick.getZRotation());
        previousTarget = targetSpeed;
    }
}