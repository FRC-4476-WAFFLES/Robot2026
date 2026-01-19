package frc.robot.utils.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseEstimateRecord;
import frc.robot.subsystems.vision.VisionIO.RawFiducialRecord;
import frc.robot.utils.vision.LimelightHelpers.PoseEstimate;

public class VisionHelpers {
    private static final double[] DEFAULT_STDDEVS = new double[12];

    /**
     * Depending on the state of the robot, get which tags to localize off of 
     * @return an int array of valid tag IDs
     */
    public static int[] getValidTagIDs() {
        // Expand to allow other tags while not coral pathing, but for now is ok
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return VisionConstants.RED_VALID_REEF_TAG_IDs;
            }
        }
        return VisionConstants.BLUE_VALID_REEF_TAG_IDs;
    }

    /**
    * Calculates the standard deviations for a given limelight pose estimate
    * For use with Megatag 1
    */
    public static Matrix<N3, N1> calculateStdDevsMegatag(PoseEstimateRecord limelightPoseEstimate,
            RawFiducialRecord[] rawFiducials) {
        Matrix<N3, N1> estStdDevs;

        if (limelightPoseEstimate.tagCount() == 1) {
            estStdDevs = VisionConstants.defaultkSingleTagStdDevsMT1;

            // Reduce the contribution of ambiguous tags
            estStdDevs.times(getMegatagEstimateQuality(rawFiducials));
        } else {
            // If multiple tags are visible, use different (lower) deviations
            estStdDevs = VisionConstants.defaultMultiTagStdDevsMT1;
        }

        if (limelightPoseEstimate.tagCount() == 0) {
            return estStdDevs;
        }

        // Scale based on distance
        estStdDevs = estStdDevs
                .times(1 + (limelightPoseEstimate.avgTagDist() * limelightPoseEstimate.avgTagDist() / 6));

        return estStdDevs;
    }

    /**
     * Calculates the standard deviations for a given limelight pose estimate being fused with gyro orientation
     * For use with Megatag 1 + Gyro estimates
     */
    public static Matrix<N3, N1> calculateStdDevsGyroFusion(PoseEstimateRecord limelightPoseEstimate) {
        Matrix<N3, N1> estStdDevs = VisionConstants.defaultStdDevsFusedGyroEstimate;

        if (limelightPoseEstimate.tagCount() == 0) {
            return estStdDevs;
        }

        // Lightly scale based on distance
        estStdDevs = estStdDevs
                .times(1 + (limelightPoseEstimate.avgTagDist() * limelightPoseEstimate.avgTagDist() / 50));

        return estStdDevs;
    }

    public static double[] getAutomaticStandardDeviations(String limelightName) {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("stddevs")
                .getDoubleArray(DEFAULT_STDDEVS);
    }

    public static double getMegatagEstimateQuality(PoseEstimate limelightPoseEstimate) {
        return 1 / (1 - limelightPoseEstimate.rawFiducials[0].ambiguity);
    }

    public static double getMegatagEstimateQuality(RawFiducialRecord[] rawFiducials) {
        return 1 / (1 - rawFiducials[0].ambiguity());
    }

    /**
     * Validates that a pose estimate contains valid values and is reasonable
     * @param pose The pose to validate
     * @return true if the pose is valid, false otherwise
     */
    public static boolean isValidPose(Pose2d pose) {
        if (pose == null) {
            return false;
        }

        // Check for NaN/infinite values
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) ||
                Double.isNaN(pose.getRotation().getDegrees()) ||
                !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY()) ||
                !Double.isFinite(pose.getRotation().getDegrees())) {
            return false;
        }

        // Check if pose is too close to field origin (common vision failure)
        return pose.getTranslation().getNorm() >= VisionConstants.MIN_POSE_DISTANCE_FROM_ORIGIN;
    }

    public static boolean isValidStdevs(Matrix<N3, N1> standardDeviation) {
        return isValidNumber(standardDeviation.get(0, 0)) &&
                isValidNumber(standardDeviation.get(1, 0)) &&
                isValidNumber(standardDeviation.get(2, 0));

    }

    public static boolean isValidNumber(double num) {
        return !Double.isNaN(num) && Double.isFinite(num);
    }
}
