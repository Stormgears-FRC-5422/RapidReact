package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveDistance extends CommandBase {
    StormDrive m_drive;
    double m_distance;

    public DriveDistance(StormDrive drive,double distance) {
        m_drive = drive;
        m_distance = distance;
        addRequirements(drive);
    }

    public void initialize() {
        m_drive.getDifferentialDrive().setSafetyEnabled(false);
        m_drive.setBrakeMode();
        m_drive.resetPosition();
        m_drive.setMaxAccel(.3);
        m_drive.setMaxVelocity(1);
    }
    public void execute() {
        m_drive.setPositionReference(m_distance);
        SmartDashboard.putString("DriveDistance", "execute");
        SmartDashboard.putNumber("Distance", m_drive.getDistance());
    }

    public void end(boolean interrupted) {
        m_drive.getDifferentialDrive().arcadeDrive(0, 0);
        m_drive.getDifferentialDrive().setSafetyEnabled(true);
        m_drive.setCoastMode();  // should restore the mode instead of assuming
        SmartDashboard.putString("DriveDistance", "finished");
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



