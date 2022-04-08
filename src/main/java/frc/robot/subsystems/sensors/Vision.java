package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase implements Loggable {
    private final PhotonCamera UpperHubCam = new PhotonCamera("UpperHubCam");
    private PhotonTrackedTarget UpperHubTarget;

    public Vision() {
        System.out.println("Vision... ");
    }

    @Override
    public void periodic() {
        UpperHubTarget = UpperHubCam.getLatestResult().getBestTarget();
    }

    @Log(name="Upper Hub Relative Yaw")
    public double getUpperHubYaw() {
        return hasTarget() ? UpperHubTarget.getYaw() : 0;
    }

    @Log(name="hasTarget()")
    public boolean hasTarget() {
        return UpperHubCam.getLatestResult().hasTargets();
    }

    @Log(name="Upper Hub Area")
    public double getUpperHubArea() {
        return hasTarget()? UpperHubTarget.getArea() : 0;
    }
}
