// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static Mode simMode = Mode.SIM;

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : simMode;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* CAN IDs  */
  public static class CANIds {
    // Drivetrain IDS are located in TunerConstants

    // Other Motors
    public static final int CANdle = 22;

    // Canivore
    public static final String CANivoreName = "Drivetrain";
  }

  /* PWM Outputs */
  public static class PWMOutputs {
    // we should have none
  }

  /* Digital Ports */
  public static class DigitalOutputs {

  }

  /* Code */
  public static class CodeConstants {
    public static final double PERIODIC_LOOP_TIME = 0.02;
    public static final double TELEMETRY_LOOKBACK_TIME = 1; // s

    public static final int SUBSYSTEM_NT_UPDATE_RATE = 20; // How many times a second subsystems will publish to NT. Reduce if performance is suffering.

    // Disable all nonessential CAN status signals, potentially reducing CAN pressure
    public static final boolean DISABLE_UNUSED_STATUS_SIGNALS = true;

    // Frequencies in hertz for CAN refresh rates
    public static final double FD_CAN_FREQUENCY = 100;
    public static final double BASE_CAN_FREQUENCY = 50;
    public static final double LOW_IMPORTANCE_CAN_FREQUENCY = 20;

    public static final double AUTO_MAX_SPEED = 2;

    public static final boolean USE_PATHPLANNER_AUTOS = false;
    public static final boolean RESET_ODOMETRY_AUTO_START = true;
  }

  /* Vision */
  public static class VisionConstants {
    // Used in place of Double.maxValue to stay far away from under/overflows when performing arithematic
    public static final double LARGE_VARIANCE = 1e7;

    public static final Matrix<N3, N1> defaultkSingleTagStdDevsMT1 = VecBuilder.fill(0.04, 0.04, 4);
    public static final Matrix<N3, N1> defaultMultiTagStdDevsMT1 = VecBuilder.fill(0.02, 0.02, 3);

    public static final Matrix<N3, N1> defaultStdDevsFusedGyroEstimate = VecBuilder.fill(0.03, 0.03, LARGE_VARIANCE);

    public static final Matrix<N3, N1> defaultStdDevsMT2 = VecBuilder.fill(0.01, 0.01, LARGE_VARIANCE);

    // Number of frames to skip processing while disabled to prevent overheating
    public static final int LIMELIGHT_DISABLED_THROTTLE = 80;

    // Use standard deviations reported by the limelight as opposed to hand calculating them
    public static final boolean USE_AUTOMATIC_STANDARD_DEVIATIONS = true;

    public static final int SEDING_LL_IMU_MODE = 1; // Enables seeding
    public static final int MOVING_LL_IMU_MODE = 2; // Uses internal IMU

    public static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(0.35, -0.35, 0.2),
        new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(20))
    );
    public static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(0.35, 0.35, 0.2),
        new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(-20))
    );

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Vision validation thresholds
    public static final double AMBIGUITY_THRESHOLD = 0.7; // Max ambiguity for single tag (0-1, lower is better), 0.19 is what 254 used
    public static final double MIN_TAG_AREA_SINGLE_TAG = 1.0; // Minimum tag area (% of image, 0-100 scale) for single tag
    public static final double MIN_TAG_AREA_FOR_YAW_CHECK = 1.9; // Tag area threshold (% of image) for yaw validation
    public static final double MAX_Z_ERROR = 0.2; // Maximum acceptable Z-axis error in meters (robot should be on ground)
    public static final double MAX_YAW_DIFFERENCE_DEG = 5.0; // Max degrees difference between vision and odometry yaw for single tag
    public static final double MIN_POSE_DISTANCE_FROM_ORIGIN = 1.0; // Minimum distance from field origin (0,0) in meters
    public static final double MEGATAG1_MAX_DISTANCE_THRESHOLD = 1; // Max distance at which MT1 estimates are used raw from cameras
    public static final double MAX_YAW_RATE_RADS = 5.0;

    // Names of limelights
    public static final String LIMELIGHT_NAME_L = "limelight-right";
    public static final String LIMELIGHT_NAME_R = "limelight-left";

    // Limelights are considered disconnected if their heartbeat value is older than this many seconds
    public static final double LL_HEARTBEAT_MIN_FREQ = 0.5;

    // Used to read from the raw stddevs array returned by a limelight
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;
  }

  /* Physical */
  public static class PhysicalConstants {
    // In number of motor rotations per mechanism rotation
    public static final double exampleReduction = 7.1111;
  }
}