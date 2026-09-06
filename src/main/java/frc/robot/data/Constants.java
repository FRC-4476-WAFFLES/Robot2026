// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.lib.Spline1D.NodePoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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

    public static final int SUBSYSTEM_NT_UPDATE_RATE = 20; // How many times a second subsystems will publish to NT.
                                                           // Reduce if performance is suffering.

    // Disable all nonessential CAN status signals, potentially reducing CAN
    // pressure
    public static final boolean DISABLE_UNUSED_STATUS_SIGNALS = false;

    // Frequencies in hertz for CAN refresh rates
    public static final double FD_CAN_FREQUENCY = 100;
    public static final double BASE_CAN_FREQUENCY = 50;
    public static final double LOW_IMPORTANCE_CAN_FREQUENCY = 20;

    public static final double ON_BUMP_TILT = 9.5; // Degrees, how much off vertical axis is considered the bump

    public static final double AUTO_MAX_SPEED = 3.5;
    public static final double AUTO_MAX_ACCEL = 15.0;
    public static final double AUTO_MAX_ACCEL_BUMP = 7.0;
    public static final double AUTO_MAX_JERK = 15.0;

    public static final double AUTO_SLEW_LIMIT = 12; // Smoothes out pure pursit segments

    public static final double SOTM_SLEW_LIMIT = 5;
    public static final double SOTM_ANGLE_SLEW_LIMIT = 4;

    public static final double INTAKE_SHOOT_SLEW_LIMIT = 3.5;
    public static final double INTAKE_SHOOT_ANGLE_SLEW_LIMIT = 2;

    public static final boolean USE_PATHPLANNER_AUTOS = false;
    public static final boolean RESET_ODOMETRY_AUTO_START = true;
    public static final boolean DISABLE_PURE_PURSUIT = true;

    public static final boolean USE_FUEL_SIMULATION = false;
    public static final boolean USE_VISION_SIMULATION = true;

    public static final boolean COMPLEX_AUTO_PREVIEW = true;

    public static final Distance AUTO_POSITION_TOLERANCE_VAGUE = Meters.of(0.25);
    public static final Distance AUTO_POSITION_TOLERANCE_PRECISE = Centimeters.of(4);
    public static final Distance AUTO_MAX_TRACKING_ERROR = Meters.of(0.5);

    public static final Angle AUTO_ANGLE_TOLERANCE_PRECISE = Degrees.of(1);

    public static final NodePoint[] TimeofFlightMap = new NodePoint[] {
        new NodePoint(1.8, 0.9),
        new NodePoint(3.175, 1.15),
        new NodePoint(4.778, 1.25),
        new NodePoint(5.4, 1.4),
        new NodePoint(10, 2)
    };
    public static final double MIN_TOF = 0.6;
    // public static final double MAX_TOF = 0;

    public static final NodePoint[] TiltOffsetMap = new NodePoint[] {
        new NodePoint(9, 5),
        new NodePoint(13, 23)
    };

    public static final boolean LIMIT_TO_HUB_SHIFTS = true;
    public static final boolean MANUAL_SHOOTER_TUNING = true;
    public static final boolean SHOOT_ON_MOVE = true;

    public enum ManualOverrideTarget {
      FRONT_CLOSE(Rotation2d.kZero, 3.2),
      TRENCH(Rotation2d.kZero, 5),
      PASS(Rotation2d.kZero, 8);

      private final Rotation2d turretSetpoint; // Robot relative
      private final double distance;

      ManualOverrideTarget(Rotation2d turretSetpoint, double distance) {
        this.turretSetpoint = turretSetpoint;
        this.distance = distance;
      }

      public Rotation2d getTurretSetpoint() {
        return turretSetpoint;
      }

      public double getDistance() {
        return distance;
      }
    }
  }

  /* Vision */
  public static class VisionConstants {
    // Used in place of Double.maxValue to stay far away from under/overflows when
    // performing arithematic
    public static final double LARGE_VARIANCE = 1e7;

    public static final Matrix<N3, N1> defaultkSingleTagStdDevsMT1 = VecBuilder.fill(0.04, 0.04, 4);
    public static final Matrix<N3, N1> defaultMultiTagStdDevsMT1 = VecBuilder.fill(0.02, 0.02, 3);

    public static final Matrix<N3, N1> defaultStdDevsFusedGyroEstimate = VecBuilder.fill(0.03, 0.03, LARGE_VARIANCE);

    public static final Matrix<N3, N1> defaultStdDevsMT2 = VecBuilder.fill(0.01, 0.01, LARGE_VARIANCE);

    // Number of frames to skip processing while disabled to prevent overheating
    public static final int LIMELIGHT_DISABLED_THROTTLE = 120;

    // Use standard deviations reported by the limelight as opposed to hand
    // calculating them
    public static final boolean USE_AUTOMATIC_STANDARD_DEVIATIONS = true;
    // Ignore single tag estimates
    public static final boolean IGNORE_SINGLE_TAG = true;

    public static final int SEDING_LL_IMU_MODE = 1; // Enables seeding
    public static final int MOVING_LL_IMU_MODE = 2; // Uses internal IMU

    public static String APRITL_TAG_MAP_NAME = "";
    // Default option
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = loadDefaultFieldMap();

    // Practice Red
    // public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
    // loadFieldMap("fieldmaps/BGConlyred.json");

    // Vision validation thresholds
    public static final double AMBIGUITY_THRESHOLD = 0.7; // Max ambiguity for single tag (0-1, lower is better), 0.19
                                                          // is what 254 used
    public static final double MIN_TAG_AREA_SINGLE_TAG = 1.0; // Minimum tag area (% of image, 0-100 scale) for single
                                                              // tag
    public static final double MIN_TAG_AREA_FOR_YAW_CHECK = 1.9; // Tag area threshold (% of image) for yaw validation
    public static final double MAX_Z_ERROR = 0.2; // Maximum acceptable Z-axis error in meters (robot should be on
                                                  // ground)
    public static final double MAX_YAW_DIFFERENCE_DEG = 5.0; // Max degrees difference between vision and odometry yaw
                                                             // for single tag
    public static final double MIN_POSE_DISTANCE_FROM_ORIGIN = 1.0; // Minimum distance from field origin (0,0) in
                                                                    // meters
    public static final double MAX_YAW_RATE_RADS = 5.0;
    public static final double MAX_YAW_RATE_RADS_GYRO_ESTIMATE = 1.5;
    public static final double MAX_TURRET_YAW_RATE_ROTATIONS = 2;

    // Names of limelights
    public static final String LIMELIGHT_NAME_FRAME = "limelight-frame";
    public static final String LIMELIGHT_NAME_TURRET = "limelight-turret";

    // Limelights are considered disconnected if their heartbeat value is older than
    // this many seconds
    public static final double LL_HEARTBEAT_MIN_FREQ = 0.5;

    // Pose agreement tracking. Counts consecutive accepted vision estimates that
    // land within POSE_AGREEMENT_EPSILON of where odometry already thinks we are;
    // once that count reaches the threshold the estimate has converged and the
    // pose can be trusted for precise actions.
    public static final double POSE_AGREEMENT_EPSILON = 0.20; // meters
    public static final int POSE_STABLE_UPDATE_THRESHOLD = 100; // ~2s at 50Hz
    // Agreement goes stale if no estimate has been accepted for this long
    public static final double POSE_AGREEMENT_STALE_TIME = 0.5; // seconds

    /**
     * How far vision and odometry must disagree, and for how long, before the
     * robot is taken to have lost track of where it is.
     *
     * <p>
     * Replaying the ONWEL logs, a metre held for a second fires about once every
     * two matches, lasts a couple of seconds, and occasionally runs to sixteen.
     * That matches what the drive team reports. It is also defensible against the
     * distribution: the 99th percentile agreement error is 1.64 m, so a full
     * metre sustained is genuinely abnormal rather than the usual tail.
     *
     * <p>
     * Note this asks the opposite question to {@link #POSE_STABLE_UPDATE_THRESHOLD},
     * and deliberately. "Have the last hundred estimates all been good" is
     * unreachable when 19 % of estimates miss by 20 cm, and reports the pose lost
     * for most of a match. "Has it been badly wrong for a while" does not.
     */
    public static final double POSE_LOST_DISTANCE = 1.0; // meters
    public static final double POSE_LOST_TIME = 1.0; // seconds

    // Used to read from the raw stddevs array returned by a limelight
    public static final int MEGATAG_1_XStdDevIndex = 0;
    public static final int MEGATAG_1_YStdDevIndex = 1;
    public static final int MEGATAG_1_YawStdDevIndex = 5;

    public static AprilTagFieldLayout loadFieldMap(String path) {
      AprilTagFieldLayout layout = loadDefaultFieldMap();
      try {
        var loadedLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve(path));
        List<AprilTag> loadedTags = loadedLayout.getTags();

        // Handle partial tag layout
        for (var tag : layout.getTags()) {
          if (!loadedTags.stream().anyMatch(chosenTag -> chosenTag.ID == tag.ID)) {
            loadedTags.add(tag);
          }
        }

        layout = new AprilTagFieldLayout(loadedTags, layout.getFieldLength(), layout.getFieldWidth());

        APRITL_TAG_MAP_NAME = path;
      } catch (Exception e) {
        System.out.println("Error reading fieldmap");
        e.printStackTrace();
      }
      return layout;
    }

    public static AprilTagFieldLayout loadDefaultFieldMap() {
      APRITL_TAG_MAP_NAME = "k2026RebuiltAndymark";
      return AprilTagFieldLayout
          .loadField(AprilTagFields.k2026RebuiltAndymark);
    }
  }

  /* Physical */
  public static class PhysicalConstants {
    // In number of motor rotations per mechanism rotation
    public static final double TURRET_REDUCTION = 19.5556;
    public static final double EXPANDER_REDUCTION = 111.2142825;
    public static final double FLYWHEEL_REDUCTION = 1;
    public static final double INDEXER_REDUCTION = 5.33333;
    public static final double INTAKE_REDUCTION = 3;
    public static final double HOOD_REDUCTION = 73.3333;
    public static final double FEEDER_REDUCTION = 1.5333333;
    public static final double CLIMBER_REDUCTION = 2;

    public static final double TURRET_GEAR_TEETH = 160.0;
    public static final double ENCODER_0_TEETH = 35.0;
    public static final double ENCODER_1_TEETH = 36.0;

    public static final double TURRET_ENCODER_0_REDUCTION = TURRET_GEAR_TEETH / ENCODER_0_TEETH;
    public static final double TURRET_ENCODER_1_REDUCTION = TURRET_GEAR_TEETH / ENCODER_1_TEETH;

    // Height is applied megatag side, this is converted to a translation2d where
    // needed
    public static final Transform3d ROBOT_TO_TURRET_CENTER = new Transform3d(
        new Translation3d(-0.07302500, 0.22542500, 0.49093120),
        new Rotation3d(0, 0, 0)
    );
    // Turret-space offset, does not change with robot frame rotation
    public static final Transform3d TURRET_CAMERA_OFFSET_FROM_CENTER = new Transform3d(
        new Translation3d(-0.09576931, 0, 0.24856043),
        new Rotation3d(0, Units.degreesToRadians(-28.1), 0)
    );

    public static final Transform3d TURRET_CAMERA_OFFSET_FROM_CENTER_PARTIAL = new Transform3d(
        TURRET_CAMERA_OFFSET_FROM_CENTER.getTranslation(),
        new Rotation3d(0, 0, 0)
    );

    // Pitch & height are applied megatag side
    public static final Transform3d ROBOT_TO_FRAME_CAMERA = new Transform3d(
        new Translation3d(-0.1757, 0.40083964, 0.26252020),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(90))
    );

    // Should technically not include height on real robot but for sim it's easier
    // to debug this way
    public static final Transform3d ROBOT_TO_FRAME_CAMERA_PARTIAL = new Transform3d(
        ROBOT_TO_FRAME_CAMERA.getTranslation(),
        new Rotation3d(0, 0, ROBOT_TO_FRAME_CAMERA.getRotation().getZ())
    );
    public static final Distance FULL_WIDTH = Meters.of(0.762);
    public static final Distance FULL_LENGTH = Meters.of(0.6604);
    public static final Distance BUMPER_HEIGHT = Meters.of(0.1);

  }

  public static class TurretConstants {
    public static final double CANCODER_0_OFFSET = -0.670166015625;
    public static final double CANCODER_1_OFFSET = -0.10791015625;

    public static final Rotation2d PHYSICAL_ZERO = Rotation2d.fromDegrees(-45); // Facing diagonally back into robot

    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(-190); // Can be up to +/- 360 deg
                                                                                        // without breaking logic
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(190);

    public static final double MAX_VELOCITY = 2;
    public static final double MAX_ACCELERATION = 10;

    public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(12);

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 70;
    public static final double MOTOR_kD = 4;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 4;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;
  }

  public static class FlywheelConstants {
    public static final double MAX_VELOCITY = 10;
    public static final double MAX_ACCELERATION = 20;

    public static final double ZERO_DUTY_CYCLE = 0.25;
    public static final double ZERO_POSITION = 0;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;
    public static final double MOTOR_SUPPLY_CURRENT_LIMIT = 60;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    public static final double OFFSET = -4.5;
    // In the format of x -> distance (m), y -> flywheel speed (rps)
    public static final NodePoint[] DistanceMap = new NodePoint[] {
        new NodePoint(1.5, 45.5 + OFFSET),
        new NodePoint(1.884, 45.8 + OFFSET),
        new NodePoint(2.33, 49.0 + OFFSET),
        new NodePoint(3.05, 55.0 + OFFSET),
        new NodePoint(3.827, 59.5 + OFFSET),
        new NodePoint(4.31, 61.5 + OFFSET),
        new NodePoint(5.28, 62.0 + OFFSET),
        new NodePoint(5.6, 64.0 + OFFSET),
        new NodePoint(5.912, 65.7 + OFFSET),
        new NodePoint(7.022, 70.0 + OFFSET),
        new NodePoint(7.308, 71.9 + OFFSET), // taegen memes
        new NodePoint(12.5, 65.5 + OFFSET) // Super long passing
    };

    /**
     * The flywheel's share of the range error budget, in metres either side of
     * the target.
     *
     * <p>
     * The goal is about a metre across, so the whole budget is 0.5 m either side.
     * The flywheel does not get all of it: turret aim, pose estimate, hood angle
     * and the shoot-on-move correction all spend from the same metre. Splitting
     * it evenly leaves 0.35 m each, which combine to about 0.49 m — right at the
     * edge of the goal, and the most the flywheel can claim without the rest
     * pushing shots out the back.
     *
     * <p>
     * This is the number to tune, because it means something physical. Range
     * error is {@code 2 * distance * speedError / speed}, so a tolerance derived
     * from it scales correctly with distance instead of being wrong at both ends.
     * Raise it if the gate holds fire more than the drive team can live with;
     * lower it only if something else in the chain gets tighter first.
     */
    public static final double ACCEPTABLE_RANGE_ERROR = 0.35;
    /**
     * Bounds on the derived tolerance, in rotations per second.
     *
     * <p>
     * The lower bound is set so it does not bind inside the range shots are taken
     * from — at 5 m the derived value is 2.0 rps, so anything higher would quietly
     * widen the range error rather than hold it. The consequence is that the gate
     * is genuinely strict on long shots, which is the intent: those were landing
     * 3 to 4 m short. The upper bound catches a missing or nonsense distance.
     */
    public static final double MIN_VELOCITY_TOLERANCE = 2.0;
    public static final double MAX_VELOCITY_TOLERANCE = 8.0;
    /**
     * How long the wheel must be outside tolerance before the gate closes.
     *
     * <p>
     * Without this the gate shuts after every ball, because a ball drags the
     * wheel down a median of 7.8 rps on its way out — after it has already gone.
     * Replaying the logs, a gate with no falling debounce is open 9 % of the time
     * a shot is wanted; at 0.20 s it is open 64 %, and past 0.30 s it stops
     * improving and starts tolerating genuine sag. Normal recovery is 0.07 s, so
     * this ignores a ball dip and still catches a battery that cannot keep up.
     */
    public static final double READY_FALLING_DEBOUNCE = 0.25;
    /** How long the wheel must be inside tolerance before the gate opens. */
    public static final double READY_RISING_DEBOUNCE = 0.25;

    /**
     * The old fixed tolerance, 20 rps. Far too loose to gate a hub shot — it
     * permits a 3.6 m range error at 3.5 m — but retained for passing, where the
     * target is a region of floor rather than a goal.
     */
    public static final double RPM_RANGE = 1200; // Range before indexer will start

  }

  public static class SpindexerConstants {
    public static final double TEST_VELOCITY = 30.0;

    public static final double ZERO_DUTY_CYCLE = 0.25;
    public static final double ZERO_POSITION = 0;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    public enum IndexerState {
      RUN(13.5, 65),
      RUNSLOW(13.5, 65),
      STOP(0, 0),
      REVERSE(-1, -1);

      private final double spindexerSpeed;
      private final double conveyorSpeed;

      IndexerState(double spindexerSpeed, double conveyorSpeed) {
        this.spindexerSpeed = spindexerSpeed;
        this.conveyorSpeed = conveyorSpeed;
      }

      public double getSpindexerSpeed() {
        return spindexerSpeed;
      }

      public double getConveyorSpeed() {
        return conveyorSpeed;
      }
    }
  }

  public static class ExpanderConstants {
    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(0);
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(270);

    public static final double MAX_VELOCITY = 5;
    public static final double MAX_ACCELERATION = 5;

    public static final double ZERO_DUTY_CYCLE = -0.25;
    public static final double ZERO_POSITION = 0;
    public static final double ZERO_TORQUE_CURRENT = -40;
    public static final double ZERO_DEBOUNCE = 0.2;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 80;
    public static final double MOTOR_kD = 1;
    public static final double MOTOR_kS = 1;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    public static final double AGITATION_CYCLE_TIME = 1.5;

    public static final double SMART_AGITATION_REBOUND_DEGREES = 20;
    public static final double SMART_AGITATION_DONE_THRESHOLD_DEG = 10;
    public static final double SMART_AGITATION_STOPPED_VELOCITY = 0.02; // output shaft rot/sec
    public static final double SMART_AGITATION_EXTEND_TIME = 0.4; // seconds in extend phase
    public static final double SMART_AGITATION_MIN_RETRACT_TIME = 0.3; // min time after motion starts before checking
                                                                       // velocity

    // Configured positions
    public enum ExpanderPosition {
      STOWED(0.0),
      EXTENDED(97.0),
      INTAKING(97.0),
      AGITATION_MID(75),
      OUTTAKE(60),
      AGITATION_MAX(40);

      private final double degrees;

      ExpanderPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
  }

  public static class IntakeConstants {
    public static final double INTAKE_DUTY_CYCLE = 1; // Was 28rps with velocity control
    public static final double AGITATION_DUTY_CYCLE = 0.3; // Was 5 rps with velocity control
    public static final double OUTTAKE_DUTY_CYCLE = -1;

    // Speed-scaled intake: ramps from MIN_DUTY_SCALE at rest to 1.0 at
    // FULL_SPEED_THRESHOLD
    public static final double INTAKE_MIN_DUTY_SCALE = 0.65; // Fraction of INTAKE_DUTY_CYCLE when stationary
    public static final double INTAKE_FULL_SPEED_THRESHOLD = 0.2; // Fraction of max drive speed for full intake power

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 1;
    public static final double MOTOR_kS = 0.35;
    public static final double MOTOR_kV = 0.35;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;
  }

  public static class HoodConstants {
    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(0);
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(20);

    public static final double MAX_VELOCITY = 2;
    public static final double MAX_ACCELERATION = 30;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 40;

    public static final double MOTOR_kP = 400;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0.1;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    // In the format of x -> distance (m), y -> hood angle (rotations)
    public static final NodePoint[] DistanceMap = new NodePoint[] {
        new NodePoint(1.5, 0),
        new NodePoint(1.884, 3.09),
        new NodePoint(2.33, 6.09),
        new NodePoint(3.05, 6.13),
        new NodePoint(3.827, 5.5),
        new NodePoint(5.28, 18.86),
        new NodePoint(15.5, 21) // Super long passing
    };

    public static final double ANGLE_RANGE = 2.0;
  }

  public static class ClimberConstants {
    public static final double CLIMBER_ROTATIONS = 23;

    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(0);
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(50); // something

    public static final double MAX_VELOCITY = 2;
    public static final double MAX_ACCELERATION = 2;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    public enum ClimberPosition {
      STOWED(0.0),
      EXTENDED(125.0);

      private final double degrees;

      ClimberPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
  }
}