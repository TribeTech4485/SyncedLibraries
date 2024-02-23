package frc.robot.SyncedLibraries.SystemBases;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionBase extends SubsystemBase {
  public PhotonCamera camera;
  public boolean hasTarget = false;
  public PhotonPipelineResult latestResult;
  public List<PhotonTrackedTarget> targets;
  public PhotonTrackedTarget mainTarget;
  public double[] targetXYAngles = new double[2];

  public PhotonVisionBase(PhotonCamera camera) {
    camera.setPipelineIndex(0);
    camera.setDriverMode(false);
    this.camera = camera;
  }

  @Override
  public void periodic() {
    double x = 0;
    double y = 0;
    latestResult = camera.getLatestResult();
    targets = latestResult.getTargets();
    hasTarget = latestResult.hasTargets();
    if (hasTarget) {
      mainTarget = latestResult.getBestTarget();
      x = mainTarget.getYaw();
      y = mainTarget.getPitch();


      targetXYAngles[0] = x;
      targetXYAngles[1] = y;
    }

    SmartDashboard.putNumber("TargetX", x);
    SmartDashboard.putNumber("TargetY", y);
  }

  protected PhotonTrackedTarget containsTarget(List<PhotonTrackedTarget> targets, int... fiducialIds) {
    for (int i = 0; i < targets.size(); i++) {
      for (int j = 0; j < fiducialIds.length; j++) {
        if (targets.get(i).getFiducialId() == fiducialIds[j]) {
          return targets.get(i);
        }
      }
    }
    return null;
  }
}
