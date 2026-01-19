package frc.robot.utils.obsolete;
// package frc.robot.utils.vision;

// import java.io.IOException;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import frc.robot.Robot;
// import frc.robot.data.Constants.VisionConstants;

// /*
//  * This class is a bit of a mess, and is in serious need of a rewrite. It is currently unused for 2025
//  */
// public class PhotonVisionWrapper {
//     private final PhotonCamera camera;
//     private PhotonPoseEstimator photonEstimator;
//     private double lastEstTimestamp = 0;

//     // Simulation
//     private PhotonCameraSim cameraSim;
//     private VisionSystemSim visionSim;

//     @SuppressWarnings("unused")
//     public PhotonVisionWrapper(String cameraName, Transform3d camPosition) {
//         camera = new PhotonCamera(cameraName);

//         AprilTagFieldLayout fieldLayout;
//         try {

//             fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json"));

//             String alternateField = null;
//             // alternateField = "/home/lvuser/deploy/fieldlayouts/practice_field.json"; //
//             // comment this out prior to
//             // competitions.
//             // alternateField = "/home/lvuser/deploy/fieldlayouts/senior_football.json";

//             if (alternateField != null) {
//                 try {
//                     fieldLayout = new AprilTagFieldLayout(alternateField);
//                 } catch (IOException e) {
//                     DriverStation.reportError(e.toString(), e.getStackTrace());
//                 }
//             }

//             photonEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camPosition);
//             //fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
//         // camPosition
//             photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

//             // ----- Simulation
//             if (Robot.isSimulation()) {
//                 // Create the vision system simulation which handles cameras and targets on the
//                 // field.
//                 visionSim = new VisionSystemSim("main");
//                 // Add all the AprilTags inside the tag layout as visible targets to this
//                 // simulated field.
//                 visionSim.addAprilTags(fieldLayout);
//                 // Create simulated camera properties. These can be set to mimic your actual
//                 // camera.
//                 var cameraProp = new SimCameraProperties();
//                 cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
//                 cameraProp.setCalibError(0.62, 0.10);
//                 cameraProp.setFPS(15);
//                 cameraProp.setAvgLatencyMs(50);
//                 cameraProp.setLatencyStdDevMs(15);
//                 // Create a PhotonCameraSim which will update the linked PhotonCamera's values
//                 // with visible
//                 // targets.
//                 cameraSim = new PhotonCameraSim(camera, cameraProp);
//                 // Add the simulated camera to view the targets on this simulated field.
//                 visionSim.addCamera(cameraSim, camPosition);

//                 cameraSim.enableDrawWireframe(true);
//             }
//         } catch (Exception e) {
//             // Just give up I guess lol
//             System.out.println("=-=-=-=-=-=-=-=-= FAILED TO LOAD VISION, ROBOT WILL CRASH =-=-=-=-=-=-=-=-=");
//         }
//     }

//     public PhotonPipelineResult getLatestResult() {
//         var results = camera.getAllUnreadResults();
//         if (results.size() > 0) {
//             return results.get(results.size() - 1);
//         }
//         return new PhotonPipelineResult();
//     }

//     /**
//      * The latest estimated robot poses on the field from vision data. This may be
//      * empty. This should
//      * only be called once per loop.
//      *
//      * @return An {@link EstimatedRobotPose} with a list of estimated poses, estimate
//      *         timestamp, and targets
//      *         used for estimation.
//      */
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPoses() {
//         // Query the latest result from PhotonVision
//         var results = camera.getAllUnreadResults();

//         Optional<EstimatedRobotPose> visionEst = Optional.empty();
//         for (var result : results) {
//             visionEst = photonEstimator.update(result);
//             double latestTimestamp = result.getTimestampSeconds();
//             boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
//             if (Robot.isSimulation()) {
//                 visionEst.ifPresentOrElse(
//                         est -> getSimDebugField()
//                                 .getObject("VisionEstimation")
//                                 .setPose(est.estimatedPose.toPose2d()),
//                         () -> {
//                             if (newResult)
//                                 getSimDebugField().getObject("VisionEstimation").setPoses();
//                         });
//             }
//             if (newResult)
//                 lastEstTimestamp = latestTimestamp;
//         }
//         return visionEst;
//     }

//     /**
//      * The standard deviations of the estimated pose from
//      * {@link #getEstimatedGlobalPose()}, for use
//      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
//      * SwerveDrivePoseEstimator}.
//      * This should only be used when there are targets visible.
//      *
//      * @param estimatedPose The estimated pose to guess standard deviations for.
//      */
//     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
//         var estStdDevs = VisionConstants.defaultkSingleTagStdDevsMT1;
//         var targets = getLatestResult().getTargets();
//         int numTags = 0;
//         double avgDist = 0;
//         for (var tgt : targets) {
//             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//             if (tagPose.isEmpty())
//                 continue;
//             numTags++;
//             avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         if (numTags == 0)
//             return estStdDevs;
//         avgDist /= numTags;
//         // Decrease std devs if multiple targets are visible
//         if (numTags > 1)
//             estStdDevs = VisionConstants.defaultMultiTagStdDevsMT1;
//         // Increase std devs based on (average) distance
//         if (numTags == 1 && avgDist > 4)
//             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         else
//             estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

//         return estStdDevs;
//     }

//     // ----- Simulation

//     public void simulationPeriodic(Pose2d robotSimPose) {
//         visionSim.update(robotSimPose);
//     }

//     /** Reset pose history of the robot in the vision system simulation. */
//     public void resetSimPose(Pose2d pose) {
//         if (Robot.isSimulation())
//             visionSim.resetRobotPose(pose);
//     }

//     /** A Field2d for visualizing our robot and objects on the field. */
//     public Field2d getSimDebugField() {
//         if (!Robot.isSimulation())
//             return null;
//         return visionSim.getDebugField();
//     }
// }