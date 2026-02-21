// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.data.Constants.VisionConstants;

public class SimVisionIO implements VisionIO {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final PhotonCamera camera;
  private final String name;

  private final int kResWidth = 1280;
  private final int kResHeight = 800;

  /**
  * Creates a new VisionIOPhotonVisionSim.
  *
  * @param name The name of the camera.
  * @param poseSupplier Supplier for the robot pose to use in simulation.
  */
  public SimVisionIO(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.name = name;
    camera = new PhotonCamera(name);

    if (visionSim == null) {
      // Initialize vision sim
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.APRIL_TAG_FIELD_LAYOUT);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
    cameraProperties.setCalibError(0.35, 0.5);
    cameraProperties.setFPS(50);
    cameraProperties.setAvgLatencyMs(20);
    cameraProperties.setLatencyStdDevMs(5);
    cameraProperties.setExposureTimeMs(0.65);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    cameraSim.setMinTargetAreaPixels(1000);
    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, DoubleFunction<Transform3d> cameraOffset) {
    inputs.isAlive = true;

    Pose2d estimatedPose = poseSupplier.get();
    if (estimatedPose != null) {
      visionSim.update(estimatedPose);
      var transform = cameraOffset.apply(Timer.getTimestamp());
      visionSim.adjustCamera(cameraSim, transform);
      Logger.recordOutput("Vision/" + name + "/Sim Camera Position",
          new Pose3d(estimatedPose).plus(transform));
      // Logger.recordOutput("Vision/updateSimPose", estimatedPose);
    }

    // Write fake data
    var results = camera.getAllUnreadResults();
    boolean seesTarget = false;
    for (var result : results) {
      if (result.getMultiTagResult().isPresent()) {
        var multiTagResult = result.getMultiTagResult().get();
        Transform3d best = multiTagResult.estimatedPose.best;

        var pose_data = getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);

        inputs.megatagResult = new PoseEstimateRecord(
            pose_data.toPose2d(), Timer.getFPGATimestamp() - (result.metadata.getLatencyMillis() / 1000),
            result.metadata.getLatencyMillis(),
            multiTagResult.fiducialIDsUsed.size(), 0, 0,
            result.getBestTarget().getArea(), false
        );

        inputs.fiducialArrayLength = multiTagResult.fiducialIDsUsed.size();
        inputs.rawPose3d = pose_data;
        for (int i = 0; i < inputs.fiducialArrayLength; i++) {
          var f = multiTagResult.fiducialIDsUsed.get(i);
          inputs.rawFiducials[i] = new RawFiducialRecord( // Ignores most fields because most aren't strictly needed
              f,
              0,
              0,
              0,
              0,
              0,
              0
          );
        }
        seesTarget = true;

      } else if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        Transform3d best = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(bestTarget.getFiducialId())
            .get()
            .minus(Pose3d.kZero)
            .plus(bestTarget.bestCameraToTarget.inverse());

        var pose_data = getBotpose(best, 1, result, cameraSim);

        inputs.megatagResult = new PoseEstimateRecord(
            pose_data.toPose2d(), Timer.getFPGATimestamp() - (result.metadata.getLatencyMillis() / 1000),
            result.metadata.getLatencyMillis(),
            1, 0, 0,
            result.getBestTarget().getArea(), false
        );

        inputs.fiducialArrayLength = 1;
        inputs.rawFiducials[0] = new RawFiducialRecord( // Ignores most fields because most aren't strictly needed
            bestTarget.getFiducialId(),
            0,
            0,
            0,
            0,
            0,
            0
        );
        inputs.rawPose3d = pose_data;
        seesTarget = true;
      }

      if (seesTarget) {
        // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
        // MT2pitch,
        // MT2yaw]
        inputs.rawStandardDeviationArray = new double[] {
            0.7, 0.7, 0.0, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
      }
    }
    inputs.canSeeTag = seesTarget;
  }

  private Pose3d getBotpose(
      Transform3d fieldToCamera,
      int numTags,
      PhotonPipelineResult result,
      PhotonCameraSim cameraSim) {
    if (result == null || result.targets.isEmpty())
      return null;

    Optional<Transform3d> optRobotToCamera = visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
    Pose3d fieldToRobot;
    if (optRobotToCamera.isPresent() && false) {
      Transform3d cameraToRobot = optRobotToCamera.get().inverse();
      Pose3d robotPose3d = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
          .transformBy(cameraToRobot);
      fieldToRobot = robotPose3d;
    } else {
      fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
    }

    return fieldToRobot;
  }
}
