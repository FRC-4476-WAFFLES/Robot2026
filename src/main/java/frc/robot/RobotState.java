// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants.CodeConstants;

public class RobotState {
  public static enum ShooterState {
    TARGET_PASS,
    TARGET_HUB,
    TARGET_TAG, // Look at tag after crossing bump
    DISABLED
  }

  @AutoLogOutput(key = "RobotState/Shooter State")
  public ShooterState shooterState = ShooterState.TARGET_HUB;

  @AutoLogOutput(key = "RobotState/On Bump")
  public boolean onBump = false;

  @AutoLogOutput(key = "RobotState/Manual Mode")
  private boolean manualMode = false;

  @AutoLogOutput(key = "RobotState/Triggers Enabled")
  private boolean enabled = false;

  /*                       */
  /* Latency Compensation */
  /*                       */

  // Timestamps are in the timebase of Timer.getFPGATimestamp()
  private TimeInterpolatableBuffer<Pose2d> poseHistoryBuffer = TimeInterpolatableBuffer
      .createBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);
  private TimeInterpolatableBuffer<Double> yawVelocityHistoryBuffer = TimeInterpolatableBuffer
      .createDoubleBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);

  private TimeInterpolatableBuffer<Double> turretVelocityHistoryBuffer = TimeInterpolatableBuffer
      .createDoubleBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);
  private TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer
      .createBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);

  private ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds();
  private Pose2d latestPose = new Pose2d();

  public static final APConstraints autopilotConstraints = new APConstraints()
      .withAcceleration(CodeConstants.AUTO_MAX_ACCEL)
      .withJerk(CodeConstants.AUTO_MAX_JERK);

  public static final APProfile autopilotProfile = new APProfile(autopilotConstraints)
      .withErrorXY(CodeConstants.AUTO_POSITION_TOLERANCE_PRECISE)
      .withErrorTheta(CodeConstants.AUTO_ANGLE_TOLERANCE_PRECISE)
      .withBeelineRadius(Centimeters.of(8));

  private Autopilot autopilot = new Autopilot(autopilotProfile);

  public Autopilot autopilot() {
    return autopilot;
  }

  public Optional<Rotation2d> getTurretAngleTimestamp(double timestamp) {
    return turretAngleBuffer.getSample(timestamp);
  }

  public Optional<Double> getTurretVelocityTimestamp(double timestamp) {
    return turretVelocityHistoryBuffer.getSample(timestamp);
  }

  /**
   * Gets the robot pose at the given timestamp (FPGA timebase)
   */
  public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
    return poseHistoryBuffer.getSample(timestamp);
  }

  /**
   * Gets the robot yaw velocity at the given timestamp (FPGA timebase)
   */
  public Optional<Double> getYawVelocityAtTimestamp(double timestamp) {
    return yawVelocityHistoryBuffer.getSample(timestamp);
  }

  /**
   * Gets the robot velocity in robot space
   */
  public ChassisSpeeds getRobotVelocity() {
    return latestChassisSpeeds;
  }

  /**
   * Gets the robot velocity in field space
   */
  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(latestChassisSpeeds, getRotation());
  }

  /**
   * Returns the current odometry pose. Private to standardize all access through
   * RobotState
   */
  public Pose2d getPose() {
    return latestPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public boolean notMoving() {
    var vel = getRobotVelocity();
    return Math.abs(vel.vxMetersPerSecond) < 0.1 && Math.abs(vel.vyMetersPerSecond) < 0.1;
  }

  // Called once in earlyPeriodic to update odometry state
  public void updateOdometry(double timestamp, Pose2d pose, ChassisSpeeds chassisSpeeds) {
    // Less accurate than high hz odometry thread but probably good enough?
    poseHistoryBuffer.addSample(timestamp, pose);
    yawVelocityHistoryBuffer.addSample(timestamp, chassisSpeeds.omegaRadiansPerSecond);
    latestChassisSpeeds = chassisSpeeds;
    latestPose = pose;
  }

  public void updateTurret(double timestamp, Rotation2d position, double velocity) {
    turretAngleBuffer.addSample(timestamp, position);
    turretVelocityHistoryBuffer.addSample(timestamp, velocity);
  }

  public void updateEnabledState() {
    // Collect checks here once a loop since checking enabled has a mutex lock
    enabled = DriverStation.isEnabled();
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public void toggleManualMode() {
    manualMode = !manualMode;
  }

  public boolean isManualMode() {
    return manualMode;
  }

  public boolean robotEnabled() {
    return enabled;
  }

  public Trigger normalMode() {
    return new Trigger(() -> !manualMode && robotEnabled());
  }

  public Trigger manualMode() {
    return new Trigger(() -> manualMode && robotEnabled());
  }

  public Trigger shooterDisabled() {
    return new Trigger(() -> shooterState == ShooterState.DISABLED).and(normalMode());
  }

  public Trigger shooterTargetPassing() {
    return new Trigger(() -> shooterState == ShooterState.TARGET_PASS).and(normalMode());
  }

  public Trigger shooterTargetTag() {
    return new Trigger(() -> shooterState == ShooterState.TARGET_TAG).and(normalMode());
  }

  public Trigger shooterTargetsHub() {
    return new Trigger(() -> shooterState == ShooterState.TARGET_HUB).and(normalMode());
  }

  public Trigger shouldFire() {
    return new Trigger(() -> RobotContainer.flywheel.atSetpoint() &&
        RobotContainer.hood.atSetpoint()
    ).and(Controls.shootButton).and(shooterDisabled().negate()).and(normalMode());
  }

  public Trigger shouldFireManual() {
    return Controls.shootButton.and(manualMode());
  }
}
