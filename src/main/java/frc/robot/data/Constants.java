// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;

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
import edu.wpi.first.units.measure.Distance;
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

  /* CAN IDs */
  public static class CANIds {
    // Drivetrain IDS are located in TunerConstants

    // RIO bus
    public static final int expanderMotor = 10;
    public static final int intakeMotor = 11;
    public static final int hoodMotor = 12;
    public static final int flywheelMotor = 14;
    public static final int climberMotor = 13;
    public static final int indexerMotor1 = 15;
    public static final int indexerMotor2 = 16;
    public static final int feederMotor = 17;

    public static final int CANdle = 22;

    // Canivore
    public static final String CANivoreName = "CANivore";
    public static final CANBus CANivoreBus = new CANBus(CANivoreName);

    // CANivore bus
    public static final int turretMotor = 9;
    public static final int turretEncoder0 = 18;
    public static final int turretEncoder1 = 19;
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

    public static final double ON_BUMP_TILT = 5.0; // Degrees, how much off vertical axis is considered the bump

    public static final double AUTO_MAX_SPEED = 2; // Not respected by autopilot
    public static final double AUTO_MAX_ACCEL = 5.0;
    public static final double AUTO_MAX_JERK = 2.0;

    public static final boolean USE_PATHPLANNER_AUTOS = false;
    public static final boolean RESET_ODOMETRY_AUTO_START = true;

    public static final boolean USE_FUEL_SIMULATION = false;
    public static final boolean USE_VISION_SIMULATION = true;
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
    public static final int LIMELIGHT_DISABLED_THROTTLE = 80;

    // Use standard deviations reported by the limelight as opposed to hand
    // calculating them
    public static final boolean USE_AUTOMATIC_STANDARD_DEVIATIONS = true;

    public static final int SEDING_LL_IMU_MODE = 1; // Enables seeding
    public static final int MOVING_LL_IMU_MODE = 2; // Uses internal IMU

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

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
    public static final double MEGATAG1_MAX_DISTANCE_THRESHOLD = 1; // Max distance at which MT1 estimates are used raw
                                                                    // from cameras
    public static final double MAX_YAW_RATE_RADS = 5.0;
    public static final double MAX_YAW_RATE_RADS_GYRO_ESTIMATE = 1.5;
    public static final double MAX_TURRET_YAW_RATE_ROTATIONS = 2;

    // Names of limelights
    public static final String LIMELIGHT_NAME_FRAME = "limelight-frame";
    public static final String LIMELIGHT_NAME_TURRET = "limelight-turret";

    // Limelights are considered disconnected if their heartbeat value is older than
    // this many seconds
    public static final double LL_HEARTBEAT_MIN_FREQ = 0.5;

    // Used to read from the raw stddevs array returned by a limelight
    public static final int MEGATAG_1_XStdDevIndex = 0;
    public static final int MEGATAG_1_YStdDevIndex = 1;
    public static final int MEGATAG_1_YawStdDevIndex = 5;

    public static final boolean IGNORE_SINGLE_TAG = true;
  }

  /* Physical */
  public static class PhysicalConstants {
    // In number of motor rotations per mechanism rotation
    public static final double TURRET_REDUCTION = 2;
    public static final double EXPANDER_REDUCTION = 2;
    public static final double FLYWHEEL_REDUCTION = 2;
    public static final double INTAKE_REDUCTION = 2;
    public static final double HOOD_REDUCTION = 2;
    public static final double CLIMBER_REDUCTION = 2;

    public static final double TURRET_GEAR_TEETH = 400.0;
    public static final double ENCODER_0_TEETH = 35.0;
    public static final double ENCODER_1_TEETH = 36.0;

    public static final double TURRET_ENCODER_0_REDUCTION = TURRET_GEAR_TEETH / ENCODER_0_TEETH;
    public static final double TURRET_ENCODER_1_REDUCTION = TURRET_GEAR_TEETH / ENCODER_1_TEETH;

    public static final Transform3d ROBOT_TO_TURRET_CENTER = new Transform3d(
        new Translation3d(0.22542500, 0.07302500, 0.5),
        new Rotation3d(0, 0, 0)
    );
    public static final Transform3d TURRET_CAMERA_OFFSET_FROM_CENTER = new Transform3d(
        new Translation3d(-0.09576931, 0, 0.24856043),
        new Rotation3d(0, Units.degreesToRadians(-20), 0)
    );

    public static final Transform3d TURRET_CAMERA_OFFSET_FROM_CENTER_CALCULATION = new Transform3d(
        new Translation3d(-0.09576931, 0, 0.24856043),
        new Rotation3d(0, 0, 0)
    );

    public static final Transform3d ROBOT_TO_FRAME_CAMERA = new Transform3d(
        new Translation3d(0.35, -0.35, 0.2),
        new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(20))
    );

    public static final Distance FULL_WIDTH = Meters.of(0.6604);
    public static final Distance FULL_LENGTH = Meters.of(0.762);
    public static final Distance BUMPER_HEIGHT = Meters.of(0.1);

  }

  public static class TurretConstants {
    public static final double CANCODER_0_OFFSET = 0;
    public static final double CANCODER_1_OFFSET = 0;

    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(-360); // Can be up to +/- 360 deg
                                                                                        // without breaking logic
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(360);

    public static final double MAX_VELOCITY = 10;
    public static final double MAX_ACCELERATION = 20;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;
  }

  public static class FlywheelConstants {
    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(-360); // Can be up to +/- 360 deg
                                                                                        // without breaking logic
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(360);

    public static final double MAX_VELOCITY = 10;
    public static final double MAX_ACCELERATION = 20;

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

    // In the format of x -> distance (m), y -> flywheel speed (rps)
    public static final NodePoint[] DistanceMap = new NodePoint[] {
        new NodePoint(0, 30),
        new NodePoint(1, 40),
        new NodePoint(2, 50),
        new NodePoint(3, 60),
        new NodePoint(4, 70)
    };

    public static final double RPM_RANGE = 30; // Within 30 rpm of goal
  }

  public static class SpindexerConstants {
    public static final double TEST_VELOCITY = 30.0;

    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(-360); // Can be up to +/- 360 deg
                                                                                        // without breaking logic
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(360);

    public static final double MAX_VELOCITY = 10;
    public static final double MAX_ACCELERATION = 20;

    public static final double ZERO_DUTY_CYCLE = 0.25;
    public static final double ZERO_POSITION = 0;

    public static final double SHOOT_INDEXER_SPEED = 40;
    public static final double SHOOT_FEEDER_SPEED = 40;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;
  }

  public static class ExpanderConstants {
    public static final double MIN_POSITION_ROTATIONS = Units.degreesToRotations(0);
    public static final double MAX_POSITION_ROTATIONS = Units.degreesToRotations(270);

    public static final double MAX_VELOCITY = 4;
    public static final double MAX_ACCELERATION = 8;

    public static final double ZERO_DUTY_CYCLE = -0.25;
    public static final double ZERO_POSITION = 0;
    public static final double ZERO_TORQUE_CURRENT = -40;
    public static final double ZERO_DEBOUNCE = 0.2;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;

    // Configured positions
    public enum ExpanderPosition {
      STOWED(0.0),
      EXTENDED(125.0);

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
    public static final double MAX_ACCELERATION = 8;
    public static final double MAX_JERK = 200;

    // Motor configs
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 120;

    public static final double MOTOR_kP = 0;
    public static final double MOTOR_kD = 0;
    public static final double MOTOR_kS = 0;
    public static final double MOTOR_kV = 0;
    public static final double MOTOR_kA = 0;

    public static final double MOTOR_DEADBAND = 0;
    public static final double MOTOR_PEAK_SUPPLY_VOLTAGE = 16;
  }

  public static class HoodConstants {
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

    // In the format of x -> distance (m), y -> hood angle (rotations)
    public static final NodePoint[] DistanceMap = new NodePoint[] {
        new NodePoint(0, 1),
        new NodePoint(0.1, 1),
        new NodePoint(0.2, 1),
        new NodePoint(0.4, 1),
        new NodePoint(0.6, 1)
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