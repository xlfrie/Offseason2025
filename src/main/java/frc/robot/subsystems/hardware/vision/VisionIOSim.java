package frc.robot.subsystems.hardware.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionIOSim implements VisionIO {
  VisionSystemSim visionSystemSim = new VisionSystemSim("main");
  PhotonCamera camera;
  PhotonCameraSim cameraSim;
  List<PhotonTrackedTarget> trackedTargets;

  public VisionIOSim() {
    SimCameraProperties cameraProps = new SimCameraProperties();

    // Diagonal FOV was calculated from horizontal and vertical FOV given from https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3#hardware-specifications
    cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(74.34285844));
    cameraProps.setCalibError(0.25, 0.08);
    cameraProps.setFPS(50);
    cameraProps.setAvgLatencyMs(16);
    cameraProps.setLatencyStdDevMs(5);

    camera = new PhotonCamera("limelight");
    cameraSim = new PhotonCameraSim(camera, cameraProps);

    Rotation3d cameraRotation =
        new Rotation3d(0, Radians.of(Degrees.of(-15).in(Radian)).magnitude(), 0);
    Transform3d cameraPosition = new Transform3d(Translation3d.kZero, cameraRotation);

    visionSystemSim.addCamera(cameraSim, cameraPosition);

    cameraSim.enableProcessedStream(true);
    cameraSim.enableRawStream(true);

    cameraSim.enableDrawWireframe(true);

    try {
      visionSystemSim.addAprilTags(
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile));
    } catch (IOException ioe) {
      System.out.println("Failed to load april tags field");
    }
  }

  public void tick() {
    visionSystemSim.update(RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());

    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        trackedTargets = result.getTargets();
      }
    }
  }

  private PhotonTrackedTarget getTrackedTarget(int targetID) {
    if (trackedTargets != null && !trackedTargets.isEmpty()) {
      for (PhotonTrackedTarget trackedTarget : trackedTargets) {
        if (trackedTarget.fiducialId == targetID) {
          return trackedTarget;
        }
      }
    }
    return null;
  }

  @Override
  public Optional<Double> getTX(int targetID) {
    PhotonTrackedTarget target = getTrackedTarget(targetID);
    if (target != null)
      return Optional.of(target.getYaw());
    return Optional.empty();
  }

  @Override
  public Optional<Double> getTY(int targetID) {
    PhotonTrackedTarget target = getTrackedTarget(targetID);
    if (target != null)
      return Optional.of(target.getPitch());
    return Optional.empty();
  }
}
