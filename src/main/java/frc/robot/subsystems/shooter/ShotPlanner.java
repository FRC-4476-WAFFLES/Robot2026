// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.RobotState.ShooterState;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.data.Constants.HoodConstants;
import frc.robot.data.Constants.PhysicalConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.data.FieldConstants;
import frc.robot.subsystems.shooter.turret.Turret.TurretSetpoint;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.SplineMonotone1D;
import frc.robot.utils.lib.WafflesUtilities;

public class ShotPlanner {
  public static record ShootingParameters(
      TurretSetpoint turretSetpoint,
      double hoodAngle,
      double flywheelSpeed,
      /** Distance the shot was planned for, used to size the flywheel tolerance. */
      double distanceToTarget
  ) {}

  private static ShootingParameters parameters = null;
  private static final SplineMonotone1D flywheelSpeeds = new SplineMonotone1D(FlywheelConstants.DistanceMap);
  private static final SplineMonotone1D hoodAngle = new SplineMonotone1D(HoodConstants.DistanceMap);
  private static final SplineMonotone1D timeOfFlightMap = new SplineMonotone1D(CodeConstants.TimeofFlightMap);

  private static final SplineMonotone1D tiltOffsetMap = new SplineMonotone1D(CodeConstants.TiltOffsetMap);

  public static final Translation2d passingTargetLeft = new Translation2d(1.5, 1.5);
  public static final Translation2d passingTargetRight = new Translation2d(passingTargetLeft.getX(),
      FlippingUtil.fieldSizeY - passingTargetLeft.getY());
  public static final double latencyCompensationStep = CodeConstants.PERIODIC_LOOP_TIME;

  public static final double ACCEL_COMP_FACTOR = 0.0;
  public static final double SOTM_LOOKAHEAD = 1;
  /**
   * How many times to re-solve the lead. Five is far more than the two or three
   * it actually takes to settle, and the loop leaves early once the time of
   * flight stops moving, so the cost is a couple of map lookups.
   */
  private static final int SOTM_MAX_ITERATIONS = 5;
  /** Time of flight settling to within this is close enough, in seconds. */
  private static final double SOTM_CONVERGENCE_SECONDS = 0.005;
  /**
   * Below this range the bearing rate divides by something near zero and the
   * lead becomes meaningless. Nobody shoots from here anyway.
   */
  private static final double MINIMUM_RANGE_FOR_LEAD = 0.5;

  private static Rotation2d lastTurretAngle;
  /** When the current pose recovery attempt began, or -1 if none is running. */
  private static double recoveryStartedAt = -1;

  // private static final LoggedNetworkNumber hoodAngleTuner = new
  // LoggedNetworkNumber("/Tuning/");
  // private static final LoggedNetworkNumber shooterSpeedTuner = new
  // LoggedNetworkNumber("/Tuning/Shooter Speed");

  private static final LinearFilter turretAngleFilter = LinearFilter
      .movingAverage((int) (0.1 / CodeConstants.PERIODIC_LOOP_TIME));

  public static ShootingParameters aimAtField(Translation2d fieldTarget) { // Pose is flipped before this function
    EpochTimer.BeginEpoch("Aiming");
    {
      Pose2d robotPose = RobotContainer.state.getPose();

      ChassisSpeeds robotChassisSpeeds = RobotContainer.state.getRobotVelocity();
      robotPose = robotPose.exp(
          new Twist2d(
              robotChassisSpeeds.vxMetersPerSecond * latencyCompensationStep,
              robotChassisSpeeds.vyMetersPerSecond * latencyCompensationStep,
              robotChassisSpeeds.omegaRadiansPerSecond * latencyCompensationStep));

      Pose2d turretPose = robotPose.transformBy(
          new Transform2d(PhysicalConstants.ROBOT_TO_TURRET_CENTER.getTranslation().toTranslation2d(), Rotation2d.kZero)
      );

      double distanceToTarget = turretPose.getTranslation().getDistance(fieldTarget);

      Logger.recordOutput("RobotState/Shooter Target", new Pose2d(fieldTarget, Rotation2d.kZero));
      Logger.recordOutput("RobotState/Turret Position", turretPose);
      Logger.recordOutput("Turret/Distance To Target", distanceToTarget);

      Translation2d adjustedPose = turretPose.getTranslation();
      // Declared out here so the bearing rate below uses the same velocity the
      // lead was computed from, rather than a second estimate of it.
      Translation2d integratedVelocity = Translation2d.kZero;
      // Hastily taken from 6328. Everybody say thank you 6328.
      if (CodeConstants.SHOOT_ON_MOVE && !RobotContainer.state.onBump
      // Do not lead the shot when the turret is being used as a camera mount
      // rather than a gun: aiming at a tag, or hunting for one.
          && RobotContainer.state.shooterState != ShooterState.TARGET_TAG
          && RobotContainer.state.shooterState != ShooterState.RECOVER_POSE) {

        ChassisSpeeds robotVelocity = RobotContainer.state.getFieldVelocity();
        double robotAngle = robotPose.getRotation().getRadians();
        double turretOffsetX = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getX();
        double turretOffsetY = PhysicalConstants.ROBOT_TO_TURRET_CENTER.getY();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
            - robotVelocity.omegaRadiansPerSecond
                * (turretOffsetX * Math.sin(robotAngle) + turretOffsetY * Math.cos(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretOffsetX * Math.cos(robotAngle) - turretOffsetY * Math.sin(robotAngle));

        Translation2d turretVel = new Translation2d(turretVelocityX, turretVelocityY);
        Translation2d turretAccel = RobotContainer.state.getFieldAcceleration();
        integratedVelocity = turretVel.plus(turretAccel.times(ACCEL_COMP_FACTOR));

        // Leading the shot changes the distance, which changes the time of
        // flight, which changes the lead. Solved by iterating to a fixed point.
        //
        // This loop was written to converge and then bounded at a single pass,
        // which is the same as not converging at all: the lead was always
        // computed from the time of flight for the distance the robot is at
        // rather than the distance it will be at. The error grows with speed,
        // which is exactly when shooting on the move matters.
        double previousTimeOfFlight = Double.NaN;
        double currentDistance = distanceToTarget;
        int iterations = 0;
        for (int i = 0; i < SOTM_MAX_ITERATIONS; i++) {
          iterations = i + 1;
          double timeOfFlight = timeOfFlightMap.interpolate(currentDistance);

          adjustedPose = turretPose.getTranslation().plus(integratedVelocity.times(timeOfFlight * SOTM_LOOKAHEAD));

          currentDistance = adjustedPose.getDistance(fieldTarget);

          // The old guard read `previousTimeOfFlight != Double.NaN`, which is
          // true for every value including NaN itself, so it never meant
          // anything.
          if (!Double.isNaN(previousTimeOfFlight)
              && Math.abs(timeOfFlight - previousTimeOfFlight) < SOTM_CONVERGENCE_SECONDS) {
            break;
          }
          previousTimeOfFlight = timeOfFlight;
        }
        Logger.recordOutput("Turret/SOTM Iterations", iterations);

        distanceToTarget = currentDistance;

        Logger.recordOutput("Turret/Adjusted Robot Position", new Pose2d(adjustedPose, Rotation2d.kZero));
        Logger.recordOutput("Turret/Adjusted Distance To Target", distanceToTarget);
      }

      Rotation2d turretAngle = fieldTarget.minus(adjustedPose).getAngle();

      double turretVelocity = 0;
      if (CodeConstants.SHOOT_ON_MOVE) {
        // How fast the bearing to the target is turning, worked out from the
        // geometry rather than by differentiating the angle.
        //
        // The previous version differentiated the aim angle numerically and put
        // the result through a hundred-millisecond moving average, which cost
        // about fifty milliseconds of lag. Rotating at two radians a second that
        // is nearly six degrees of turret error, and the turret's own profile
        // then has to chase it — which is why aim suffered most while turning.
        //
        // For a stationary target and a turret moving at v, the bearing rate is
        // the component of v across the line of sight divided by the range. No
        // differentiation, no filter, no lag, and no noise to filter out.
        Translation2d lineOfSight = fieldTarget.minus(adjustedPose);
        double range = lineOfSight.getNorm();
        if (range > MINIMUM_RANGE_FOR_LEAD) {
          double bearingRateRadians = (integratedVelocity.getX() * lineOfSight.getY()
              - integratedVelocity.getY() * lineOfSight.getX())
              / (range * range);
          turretVelocity = Units.radiansToRotations(bearingRateRadians);
        }
        Logger.recordOutput("Turret/Bearing Rate", turretVelocity);
      }

      parameters = new ShootingParameters(
          new TurretSetpoint(turretAngle, turretVelocity),
          hoodAngle.interpolate(distanceToTarget),
          flywheelSpeeds.interpolate(distanceToTarget),
          distanceToTarget
      );

      // Logger.recordOutput("Turret/DEBUG ANGLE", turretAngle);
      // Logger.recordOutput("Turret/DEBUG VEL", turretVelocity);
    }
    EpochTimer.EndEpoch("Aiming");

    return parameters;
  }

  public static ShootingParameters aimToPass() {
    var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.state.getPose());
    Translation2d targetPoint = passingTargetLeft;

    if (pose.getY() > FieldConstants.fieldWidth / 2) {
      targetPoint = passingTargetRight;
    }
    return aimAtField(WafflesUtilities.FlipIfRedAlliance(targetPoint));
  }

  public static ShootingParameters aimToHub() {
    return aimAtField(WafflesUtilities.FlipIfRedAlliance(FieldConstants.Hub.topCenterPoint.toTranslation2d()));
  }

  public static ShootingParameters aimToTag() {
    return aimAtField(
        WafflesUtilities.FlipIfRedAlliance(FieldConstants.Hub.topCenterPoint.toTranslation2d())
    );
  }

  public static ShootingParameters aimManual() {
    if (CodeConstants.MANUAL_SHOOTER_TUNING) {
      parameters = new ShootingParameters(
          new TurretSetpoint(Rotation2d.kZero, 0),
          SmartDashboard.getNumber("Hood Angle", 0),
          SmartDashboard.getNumber("Shooter Speed", 0),
          0
      );

    } else {
      var target = RobotContainer.state.getManualOverrideTarget();

      parameters = new ShootingParameters(
          new TurretSetpoint(target.getTurretSetpoint(), 0),
          hoodAngle.interpolate(target.getDistance()),
          flywheelSpeeds.interpolate(target.getDistance()),
          target.getDistance()
      );
    }

    return parameters;
  }

  /**
   * Points the turret where an AprilTag should be, so vision can find one and put
   * the pose back.
   *
   * <p>
   * Aimed rather than swept, because a lost pose is usually wrong by a metre or
   * two rather than by half a field — the nearest tag by the current bad estimate
   * is very often still the nearest tag in reality, and pointing straight at it
   * finds it far faster than sweeping. If that fails to produce an agreeing
   * estimate within {@code POSE_RECOVERY_AIM_TIME}, the turret sweeps instead,
   * because at that point the estimate clearly cannot be trusted to aim with.
   *
   * <p>
   * The flywheel and hood are left at zero. This is not a shot.
   */
  public static ShootingParameters aimToRecoverPose() {
    if (recoveryStartedAt < 0) {
      recoveryStartedAt = Timer.getTimestamp();
    }
    double elapsed = Timer.getTimestamp() - recoveryStartedAt;
    Logger.recordOutput("Turret/Pose Recovery Elapsed", elapsed);

    if (elapsed > CodeConstants.POSE_RECOVERY_AIM_TIME) {
      // Sweep. A triangle wave across the turret's travel, slow enough that a
      // camera at 50 Hz gets many frames of any tag it crosses.
      double period = CodeConstants.POSE_RECOVERY_SWEEP_PERIOD;
      double phase = ((elapsed - CodeConstants.POSE_RECOVERY_AIM_TIME) % period) / period;
      double fraction = phase < 0.5 ? phase * 2 : 2 - phase * 2;
      double degrees = -CodeConstants.POSE_RECOVERY_SWEEP_DEGREES
          + fraction * 2 * CodeConstants.POSE_RECOVERY_SWEEP_DEGREES;
      Logger.recordOutput("Turret/Pose Recovery Sweeping", true);
      return new ShootingParameters(
          new TurretSetpoint(Rotation2d.fromDegrees(degrees), 0), 0, 0, 0);
    }

    Logger.recordOutput("Turret/Pose Recovery Sweeping", false);
    var aimed = aimAtField(nearestTagTranslation());
    return new ShootingParameters(aimed.turretSetpoint(), 0, 0, aimed.distanceToTarget());
  }

  /** Clears the recovery timer so the next attempt aims before it sweeps. */
  public static void resetPoseRecovery() {
    recoveryStartedAt = -1;
  }

  /** The closest AprilTag to where the robot currently believes it is. */
  private static Translation2d nearestTagTranslation() {
    Translation2d robot = RobotContainer.state.getPose().getTranslation();
    Translation2d best = robot;
    double bestDistance = Double.MAX_VALUE;
    for (var tag : FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTags()) {
      Translation2d position = tag.pose.getTranslation().toTranslation2d();
      double distance = position.getDistance(robot);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = position;
      }
    }
    Logger.recordOutput("Turret/Pose Recovery Target", new Pose2d(best, Rotation2d.kZero));
    return best;
  }

  public static ShootingParameters aimBeached() {
    parameters = new ShootingParameters(new TurretSetpoint(Rotation2d.kZero, 0), 0, 0, 0);
    return parameters;
  }

  public static ShootingParameters getLatestParameters() {
    return parameters;
  }

  public static Rotation2d getTurretBumpOffset() {
    double angle = RobotContainer.state.getLatestTilt();
    if (!RobotContainer.state.onBump) {
      return Rotation2d.kZero;
    }

    double offset = tiltOffsetMap.interpolate(angle);
    if (WafflesUtilities.FlipIfRedAlliance(RobotContainer.state.getPose()).getY() > 4) {
      // turn left?
      return Rotation2d.fromDegrees(offset);
    }
    return Rotation2d.fromDegrees(-offset);
  }

  public static Supplier<TurretSetpoint> turretSetpoint() {
    return () -> parameters.turretSetpoint;
  }

  public static DoubleSupplier flywheelSpeed() {
    return () -> parameters.flywheelSpeed;
  }

  public static DoubleSupplier hoodAngle() {
    return () -> parameters.hoodAngle;
  }

  /** Distance the current shot is planned for, or 0 if nothing is planned yet. */
  public static double distanceToTarget() {
    return parameters == null ? 0 : parameters.distanceToTarget;
  }
}
