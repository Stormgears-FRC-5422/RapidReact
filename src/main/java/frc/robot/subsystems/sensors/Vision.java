package frc.robot.subsystems.sensors;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.Constants.*;

public class Vision extends SubsystemBase implements Loggable {
    private final PhotonCamera UPPER_HUB_CAM = new PhotonCamera("UpperHubCam");
    private final double CAMERA_HEIGHT_METERS = kCameraHeightMeters;
    private final double TARGET_HEIGHT_METERS = kTaretHeightMeters;
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(kCameraPitchDeg);

    private PhotonTrackedTarget upperHubTarget;

    public Vision() {
        System.out.println("Vision... ");
    }

    private PhotonTrackedTarget getTarget() {
        return UPPER_HUB_CAM.getLatestResult().getBestTarget();
    }

    @Log(name = "Upper Hub Relative Yaw")
    public double getYaw() {
        return hasTarget() ? getTarget().getYaw() : 0;
    }

    @Log(name = "Distance To Hub")
    public double getDistance() {
        return PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getTarget().getPitch())
        );
    }

    @Log(name = "hasTarget()")
    public boolean hasTarget() {
        return UPPER_HUB_CAM.getLatestResult().hasTargets();
    }
}
