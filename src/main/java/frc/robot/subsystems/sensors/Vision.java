package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase implements Loggable {
    //TODO: make this line of code work, I think this isn't pointing to the correct name
    private final PhotonCamera UpperHubCamera = new PhotonCamera("UpperHubCamera-output");
    private PhotonTrackedTarget UpperHubTarget;

    public Vision() {
        UpperHubTarget = UpperHubCamera.getLatestResult().getBestTarget();
    }

    @Override
    public void periodic() {
        UpperHubTarget = UpperHubCamera.getLatestResult().getBestTarget();
    }

    @Log(name="Upper Hub Relative Yaw")
    public double getUpperHubYaw() {
        return hasTarget() ? UpperHubTarget.getYaw() : 0;
    }

    @Log(name="hasTarget()")
    public boolean hasTarget() {
        return UpperHubCamera.getLatestResult().hasTargets();
    }

    @Log(name="Upper Hub Area")
    public double getUpperHubArea() {
        return hasTarget()? UpperHubTarget.getArea() : 0;
    }
}
