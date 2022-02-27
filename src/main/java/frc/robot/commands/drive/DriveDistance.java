package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;

public class DriveDistance extends CommandBase {
    StormDrive m_drive;
    double m_distance;

    public DriveDistance(StormDrive drive,double distance) {
        m_drive = drive;
        m_distance = distance;
        addRequirements(drive);
    }

    public void intialize() {
        m_drive.resetPosition();

    }
    public void execute() {
        m_drive.setPositionReference(m_distance);

    }

    public void end() {
        m_drive.getDifferentialDrive().arcadeDrive(0, 0);
    }

    public boolean isFinished() {
        if ( m_drive.getDistance() > (0.98 * m_distance) && m_drive.getVelocity() < 0.0001) {
            return(true);
        }
        else {
            return(false);
        }
    }

}



