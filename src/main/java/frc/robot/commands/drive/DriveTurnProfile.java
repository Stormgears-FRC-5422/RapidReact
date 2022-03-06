package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.sensors.NavX;

public class DriveTurnProfile extends CommandBase {
    TrapezoidProfileCommand tcommand; // Must use delegation since we can't construct super (parent class) with non-static method for m_output
    StormDrive m_drive;
    NavX m_navx;
    double m_start_angle = 0.0;

    public DriveTurnProfile(TrapezoidProfile profile,StormDrive drive, NavX nav) {
        tcommand = new TrapezoidProfileCommand(profile,this::execute_pid,drive);
        m_drive = drive;
        m_navx = nav;
    }

    public DriveTurnProfile(double angle, double velocity, double acceleration,StormDrive drive, NavX nav) {
        this(new TrapezoidProfile(new TrapezoidProfile.Constraints(velocity,acceleration),new TrapezoidProfile.State(angle,0)),
                drive,
                nav);
    }

    protected double getRelativeMeasurement() {
        return(m_navx.getAngle() - m_start_angle);
    }

    public void initialize() {
        super.initialize();
        tcommand.initialize();
        m_start_angle = m_navx.getAngle();

        SmartDashboard.putString("DriveTurn","Running");
    }

    private void execute_pid(TrapezoidProfile.State state) {
        m_drive.setTurnPositionReferenceWithVelocity(getRelativeMeasurement(),state.position,state.velocity);
    }

    public void execute() {
        tcommand.execute();
    }

    public void end(boolean interrupted) {
        super.end(interrupted);
        tcommand.initialize();
        SmartDashboard.putString("DriveTurn", "finished");
    }
    public boolean isFinished() {
        return(tcommand.isFinished());
    }
}



