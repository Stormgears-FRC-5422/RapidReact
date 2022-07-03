package frc.robot.subsystems.sensors;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.*;

public class Vision extends SubsystemBase {
  private final PhotonCamera UPPER_HUB_CAM = new PhotonCamera("UpperHubCam");
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(kCameraHeightInches);
  private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(kTaretHeightInches);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(kCameraPitchDeg);

  public Vision() {
    System.out.println("... Vision");
  }

  private PhotonTrackedTarget getTarget() {
    return UPPER_HUB_CAM.getLatestResult().getBestTarget();
  }

  public double getYaw() {
    return hasTarget() ? getTarget().getYaw() : 0;
  }

  public double getArea() {
    return hasTarget() ? getTarget().getArea() : 0;
  }

  public double getDistance() {
    if (hasTarget()) {
      double pitch = getTarget().getPitch();
      return PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(pitch));
    }
    return 0;
  }

  public boolean hasTarget() {
    try {
      return UPPER_HUB_CAM.getLatestResult().hasTargets();
    } catch (Exception ignored) {
      return false;
    }
  }
}
