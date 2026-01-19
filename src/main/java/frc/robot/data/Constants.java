// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

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
    // Drive Motors
    // Drivetrain IDS are located in TunerConstants

    // public static final int steeringFrontLeft = 1; 
    // public static final int drivingFrontLeft = 2; 
    // public static final int steeringFrontRight = 3; 
    // public static final int drivingFrontRight = 4; 
    // public static final int steeringBackLeft = 5; 
    // public static final int drivingBackLeft = 6; 
    // public static final int steeringBackRight = 7; 
    // public static final int drivingBackRight = 8; 

    // // Drive Sensors
    // public static final int frontLeftAbsoluteEncoder = 9; // CANcoder
    // public static final int frontRightAbsoluteEncoder = 10; // CANcoder
    // public static final int backLeftAbsoluteEncoder = 11; // CANcoder
    // public static final int backRightAbsoluteEncoder = 12; // CANcoder

    // public static final int pidgeon = 50;

    // Other Motors
    public static final int elevator1 = 13;
    public static final int elevator2 = 14;
    public static final int manipulatorIntake = 16;
    public static final int manipulatorPivot = 15;
    public static final int groundPivotMotor = 21;
    public static final int groundIntakeMotorMid = 18;
    public static final int groundIntakeMotorRight = 19;
    public static final int groundIntakeMotorLeft = 20;
    // Other Sensors
    public static final int pivotAbsoluteEncoder = 70;

    public static final int groundIntakeLaserCanRight = 42;
    public static final int groundIntakeLaserCanMid = 40;
    public static final int groundIntakeLaserCanLeft = 41;

    public static final int manipulatorLaserCan = 44;

    public static final int groundIntakeCanRange = 17;
    public static final int CANdle = 22;

    // Canivore
    public static final String CANivoreName = "Drivetrain Backup";
  }

  /* PWM Outputs */
  public static class PWMOutputs {
    // we should have none
  }

  /* Digital Ports */
  public static class DigitalOutputs {
    public static final int coastModeSwitch = 4; // Limit Switch
    public static final int coralSensor = 9; // Coral detection sensor
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

    public static final String LIMELIGHT_NAME_CORAL = "limelight-coral";

    // Exclusively rely on reef tags for megatag
    // Prob want to expand this a lot later, but for terminal guidance this is all we should care about
    public static final int[] RED_VALID_REEF_TAG_IDs = {
        6, 7, 8, 9, 10, 11
    };

    public static final int[] BLUE_VALID_REEF_TAG_IDs = {
        17, 18, 19, 20, 21, 22
    };

    // Limelights are considered disconnected if their heartbeat value is older than this many seconds
    public static final double LL_HEARTBEAT_MIN_FREQ = 0.5;

    // Used to read from the raw stddevs array returned by a limelight
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;
  }

  /* Field */
  public static class FieldConstants {
    // Viewed from blue alliance driver station 
    public static final Translation2d HumanPlayerLeftPos = new Translation2d(1.6, 7.35);
    public static final Translation2d HumanPlayerRightPos = new Translation2d(1.6, 0.7);
  }

  /* Physical */
  public static class PhysicalConstants {
    // Placeholder values. Tune.
    // public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // public static final double maxAngularSpeed = 6; // Max Rad/s

    public static final double withBumperBotHalfWidth = 0.460; // m

    // In number of motor rotations per mechanism rotation
    public static final double groundIntakeTopRollerReduction = 7.1111;
    public static final double groundIntakeSideRollersReduction = 9.5238;
    public static final double groundPivotReduction = 32.0579;
    public static final double pivotReduction = 52.1481;
    public static final double intakeReduction = 19.4444;

    public static final double elevatorReductionToMeters = 26.6; // Motor rotations to elevator height in meters

    public static final double pivotAbsoluteEncoderOffset = -0.267822265625;
    public static final boolean usePivotAbsoluteEncoder = false; // Fallback, if false relies on internal motor encoder

    public static final Distance manipulatorWheelRadius = Inches.of(1.5);
  }

  public static class ScoringConstants {
    public static final double AUTO_SCORE_PIVOT_NUDGE = 8;
    public static final double ALGAE_TOSS_PIVOT_ANGLE = 155; // Angle at which toss occurs
    public static final double L1_HEADING_LOCK_ENGAGE_DIFFERENCE = 45; // If within 30deg of L1 angle, engage heading lock 
    public static final double L1_HEADING_LOCK_RIPOFF_VALUE = 0.2; // Break auto align if joystick value higher than this

    // Makes the elevator go up more in net autos, we can tip over but it *is* faster! :)
    public static final boolean USE_RISKY_NET_AUTO = true;
    public static final boolean USE_CORAL_SCORE_PATH_PLANNING = false; // Too slow / inconsistently latent on rio2
    public static final double SCORE_WAIT_TIME = 0.1; // Wait before driving away to allow arm to swing out

    /** A collection of scoring parameters */
    public record CoralScoringParameters(
        double maxVelocity,
        Rotation2d maxThetaVelocity,

        double maxDistanceX,
        double maxDistanceY,
        Rotation2d maxThetaDifference,

        SuperstructureState executeScoreState
    ) {}

    public static final CoralScoringParameters L4Params = new CoralScoringParameters(
        0.03,
        Rotation2d.fromDegrees(1),
        0.02,
        0.02,
        Rotation2d.fromDegrees(1.5),
        SuperstructureState.EXECUTE_L4
    );

    public static final CoralScoringParameters L3Params = new CoralScoringParameters(
        0.03,
        Rotation2d.fromDegrees(1),
        0.02,
        0.02,
        Rotation2d.fromDegrees(1.5),
        SuperstructureState.EXECUTE_L3
    );

    public static final CoralScoringParameters L2Params = new CoralScoringParameters(
        0.03,
        Rotation2d.fromDegrees(1),
        0.03,
        0.03,
        Rotation2d.fromDegrees(2),
        SuperstructureState.EXECUTE_L2
    );
  }

  /* Manipulator Constants */
  public static class ManipulatorConstants {
    // Detection thresholds
    public static final double ALGAE_CURRENT_THRESHOLD = 25.0; // amps
    public static final double CORAL_CURRENT_THRESHOLD = 20;
    public static final double ALGAE_HOLD_CURRENT_THRESHOLD = 10.0; // amps - Minimum current while holding algae

    public static final double SENSOR_DISTANCE = 115;

    public static final double ALGAE_DETECTION_DEBOUNCE_TIME = 0.15;
    public static final double CORAL_DETECTION_DEBOUNCE_TIME = 0.1;
    public static final double CORAL_RELEASE_DEBOUNCE_TIME = 0.1;
    public static final double ALGAE_HOLD_CHECK_DEBOUNCE_TIME = 0.3; // Time before checking if algae dropped while holding

    public static final double ZERO_DEBOUNCE_TIME = 0.2;

    // Intake speeds
    public static final double CORAL_INTAKE_SPEED = -5; // Rps
    public static final double ALGAE_HOLD_SPEED = -0.5; // Speed to hold algae in place
    public static final double CORAL_HOLD_SPEED = -0.2; // Speed to hold algae in place
    public static final double ALGAE_INTAKE_SPEED = -5;
    public static final double ZEROING_SPEED = -0.095; // Slow inwards speed

    // Pivot constants
    public static final double PIVOT_ANGLE_DEADBAND = 2;

    public static final double PIVOT_MIN_ANGLE = 0.0; // degrees 
    public static final double PIVOT_MAX_ANGLE = 270.0; // degrees

    public static final double PIVOT_REEF_CLEAR_ANGLE = 165.0; // degrees

    // Constraints
    public static final double PIVOT_FRAME_LOWER_CLEARANCE_ANGLE = 15;
    public static final double PIVOT_FRAME_UPPER_CLEARANCE_ANGLE = 90;
    public static final double FIRST_STAGE_AVOIDANCE_ANGLE = 215;

    public static final double PIVOT_CLEARANCE_POSITION = 38;
    public static final double PIVOT_CLEARANCE_POSITION_ALGAE = 105;

    // Motor configuration
    public static final double PIVOT_MOTION_CRUISE_VELOCITY = 1.5;
    public static final double PIVOT_MOTION_CRUISE_VELOCITY_SLOW = 0.9;
    public static final double PIVOT_MOTION_ACCELERATION = 25.0;
    public static final double PIVOT_MOTION_ACCELERATION_SLOW = 10.0;
    public static final double PIVOT_MOTION_JERK = 2000.0;
    public static final double STATOR_CURRENT_LIMIT = 50.0; // amps
    public static final double PIVOT_MOTOR_DEADBAND = 0.001;
    public static final double PIVOT_SUPPLY_VOLTAGE = 12.0;
    public static final double PIVOT_CURRENT_THRESHOLD = 27.0; // amps - Current threshold for zeroing

    // PID Values
    public static final double PIVOT_kP = 150.0;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;
    public static final double PIVOT_kS = 0.25;

    public static final double PIVOT_kG_HORIZONTAL = 0.0;

    public static final double PIVOT_kP_ALGAE_SLOW = 40.0;
  }

  /* Elevator Constants */
  public static class ElevatorConstants {
    // Controwl constants
    public static final double ELEVATOR_DEAD_ZONE = 0.015;
    public static final double ZEROING_SPEED = -0.1; // Slow downward speed
    public static final double STALL_CURRENT_THRESHOLD = 35.0; // Amperes
    public static final double ZERO_DEBOUNCE_TIME = 0.2;

    // Elevator will not move if the pivot is not past this angle, to avoid collision with top bar
    public static final double CROSSBAR_MIN_CLEAR_ANGLE = 32;
    public static final double PIVOT_BUMPER_CLEAR_HEIGHT = 0.08;

    public static final double GROUND_INTAKE_SAFETY_HEIGHT = 0.135;
    public static final double PIVOT_HITS_GROUND_INTAKE_ANGLE = 25;

    public static final double MIN_ELEVATOR_HEIGHT = 0;
    public static final double MAX_ELEVATOR_HEIGHT = 1.50;

    // Collision zone constants
    public static final double COLLISION_ZONE_LOWER = 0.36; // meters
    public static final double COLLISION_ZONE_UPPER = 0.85; // meters

    // Height where first stage starts moving
    public static final double FIRST_STAGE_START_HEIGHT = ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2.0;

    // Motion Magic configuration
    public static final double MOTION_CRUISE_VELOCITY = 6; // 4 usually
    public static final double MOTION_ACCELERATION = 4;
    public static final double MOTION_JERK = 2000;

    // PID Values
    public static final double kP = 60.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;
    public static final double kG = 0.36;

    // Motor configuration constants  
    public static final double STATOR_CURRENT_LIMIT = 80.0; // amps - Increased for better performance
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // amps - Prevent brownouts
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0; // amps - Lower limit to prevent breaker trips
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.1; // seconds - Time at lower limit
    public static final double MOTION_MAGIC_EXPO_KV = 3.0; // kV is V/rps
    public static final double MOTION_MAGIC_EXPO_KA = 0.7; // kA is V/(rps/s)

    // Simulation constants
    public static final double SIM_FREQ = 2;
    public static final double SIM_DAMPING = 1.0;
    public static final double SIM_INITIAL_POSITION = 0.0;
    public static final double SIM_INITIAL_VELOCITY = 0.0;

    // SysId constants
    public static final double SYSID_DYNAMIC_VOLTAGE = 3.0; // volts 
  }

  /* Ground Pivot Constants */
  public static class GroundPivotConstants {
    // Control constants
    public static final double DEAD_ZONE = 5.0; // In degrees
    public static final double MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double MAX_ANGLE = 205.0; // Maximum angle in degrees - adjust as needed

    // Zeroing
    public static final double ZEROING_SPEED = -0.15;
    public static final double ZERO_DEBOUNCE_TIME = 0.8; // seconds
    public static final double PIVOT_CURRENT_THRESHOLD = 16; // amps

    // Motor configuration
    public static final double STATOR_CURRENT_LIMIT = 40.0; // amps
    public static final double MOTION_CRUISE_VELOCITY = 5;
    public static final double MOTION_ACCELERATION = 10;
    public static final double MOTION_JERK = 2000.0;

    // PID Values
    public static final double kP = 25.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;

    // Predefined positions for the ground pivot (in degrees)
    public enum GroundPivotPosition {
      STOWED(0.0),
      HANDOFF(25.0),
      DEPLOYED(205.0),
      DEPLOYED_OFFGROUND(204.0),
      L1(110),
      L1_INTAKE(206),
      ZEROING_CLEARANCE(140);

      private final double degrees;

      GroundPivotPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
  }

  /* Ground Intake Constants */
  public static class GroundIntakeConstants {
    // PID Values
    public static final double kP = 60.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;
    public static final double STATOR_CURRENT_LIMIT = 60;

    public static final double CANRANGE_PROXIMITY_THRESHOLD = 0.07;
    public static final double CORAL_LEFT_DISTANCE_THRESHOLD = 45;
    public static final double CORAL_MID_DISTANCE_THRESHOLD = 30;
    public static final double CORAL_RIGHT_DISTANCE_THRESHOLD = 45;

    public static final double SENSOR_DEBOUNCE_TIME = 0.25;
  }
}