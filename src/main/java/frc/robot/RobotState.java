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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.CodeConstants.ManualOverrideTarget;
import frc.robot.data.Constants.Mode;
import frc.robot.subsystems.intake.Intake.ExpanderState;
import frc.robot.utils.vendor.HubShiftUtil;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  public static enum ShooterState {
    TARGET_PASS,
    TARGET_HUB,
    TARGET_TAG, // Look at tag after crossing bump
    HANDLE_BEACHED,
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

  @AutoLogOutput(key = "RobotState/Autonomous Enabled")
  private boolean autonomousEnabled = false;

  @AutoLogOutput(key = "RobotState/Hub Enabled")
  private boolean hubEnabled = false;

  public enum AutoWinnerOverride {
    US,
    THEM,
    NONE
  }

  @AutoLogOutput(key = "RobotState/Auto Winner Override")
  private AutoWinnerOverride autoWinnerOverride = AutoWinnerOverride.NONE;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Manual Target")
  private ManualOverrideTarget manualOverrideTarget = ManualOverrideTarget.FRONT_CLOSE;

  @Getter
  @Setter
  @AutoLogOutput(key = "Intake/Expander State")
  private ExpanderState expanderState = ExpanderState.STOWED;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Shooting")
  private boolean isShooting = false;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Intaking")
  private boolean isIntaking = false;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Outtake Desired")
  private boolean outtakeDesired = false;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Force Intake In")
  private boolean forceIntakeIn = false;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Probably Done Shooting")
  private boolean probablyDoneShooting = false;

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
  private ChassisSpeeds latestFieldSpeeds = new ChassisSpeeds();
  private Pose2d latestPose = new Pose2d();
  private Translation2d latestAcceleration = new Translation2d();

  @Getter
  private double latestTilt = 0;

  private static final APConstraints autopilotConstraints = new APConstraints()
      .withVelocity(CodeConstants.AUTO_MAX_SPEED)
      .withAcceleration(CodeConstants.AUTO_MAX_ACCEL)
      .withJerk(CodeConstants.AUTO_MAX_JERK);
  // Why doesn't APConstraints have getters?? Didn't bother subclassing to add
  // them.
  @Getter
  private static double autopilotVelocityConstraint = Double.MAX_VALUE;

  private static final APProfile autopilotProfile = new APProfile(autopilotConstraints)
      .withErrorXY(CodeConstants.AUTO_POSITION_TOLERANCE_PRECISE)
      .withErrorTheta(CodeConstants.AUTO_ANGLE_TOLERANCE_PRECISE)
      .withBeelineRadius(Centimeters.of(8));

  private Autopilot autopilot = new Autopilot(autopilotProfile);

  public static void resetAutopilotConstraints() {
    autopilotConstraints.withVelocity(CodeConstants.AUTO_MAX_SPEED);
    autopilotConstraints.withAcceleration(CodeConstants.AUTO_MAX_ACCEL);
    autopilotConstraints.withJerk(CodeConstants.AUTO_MAX_JERK);
  }

  public static void setAutopilotMaxVelocity(double velocity) {
    autopilotConstraints.withVelocity(velocity);
    autopilotVelocityConstraint = velocity;
  }

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
    return latestFieldSpeeds;
  }

  @AutoLogOutput(key = "RobotState/FieldAccel")
  public Translation2d getFieldAcceleration() {
    return latestAcceleration;
  }

  /**
   * Returns the current odometry pose. Private to standardize all access through
   * RobotState
   */
  @AutoLogOutput(key = "RobotState/FieldPose")
  public Pose2d getPose() {
    return latestPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public boolean notMoving() {
    var vel = getRobotVelocity();
    return Math.abs(vel.vxMetersPerSecond) < 0.1 && Math.abs(vel.vyMetersPerSecond) < 0.1
        && Math.abs(vel.omegaRadiansPerSecond) < 0.1;
  }

  public boolean joysticksFree(double deadband) {
    return Math.abs(Controls.getDriveXRaw()) < deadband && Math.abs(Controls.getDriveYRaw()) < deadband
        && Math.abs(Controls.getDriveRotationRaw()) < deadband;
  }

  public boolean joysticksFree() {
    return joysticksFree(0.1);
  }

  // Called once in earlyPeriodic to update odometry state
  public void updateOdometry(double timestamp, Pose2d pose, ChassisSpeeds chassisSpeeds, double tilt) {
    // Less accurate than high hz odometry thread but probably good enough?
    poseHistoryBuffer.addSample(timestamp, pose);
    yawVelocityHistoryBuffer.addSample(timestamp, chassisSpeeds.omegaRadiansPerSecond);
    latestChassisSpeeds = chassisSpeeds;

    var newSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(latestChassisSpeeds, getRotation());

    latestAcceleration = new Translation2d(
        MathUtil.clamp(
            (newSpeeds.vxMetersPerSecond - latestFieldSpeeds.vxMetersPerSecond) / CodeConstants.PERIODIC_LOOP_TIME, -6,
            6),
        MathUtil.clamp(
            (newSpeeds.vyMetersPerSecond - latestFieldSpeeds.vyMetersPerSecond) / CodeConstants.PERIODIC_LOOP_TIME, -6,
            6)
    );
    latestFieldSpeeds = newSpeeds;
    latestPose = pose;

    latestTilt = tilt;
    if (Constants.getMode() == Mode.SIM) {
      if (onBump) {
        latestTilt = 16;
      }
    }
  }

  public void updateTurret(double timestamp, Rotation2d position, double velocity) {
    turretAngleBuffer.addSample(timestamp, position);
    turretVelocityHistoryBuffer.addSample(timestamp, velocity);
  }

  @SuppressWarnings("unused")
  public void updateEnabledState() {
    // Collect checks here once a loop since checking enabled has a mutex lock
    enabled = DriverStation.isEnabled();
    autonomousEnabled = DriverStation.isAutonomousEnabled();

    // checking this is kind of slow so aggregate
    hubEnabled = !CodeConstants.LIMIT_TO_HUB_SHIFTS || HubShiftUtil.getShiftedShiftInfo().active();
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public void toggleManualMode() {
    manualMode = !manualMode;
    SmartDashboard.putBoolean("Manual", manualMode);
  }

  public boolean isManualMode() {
    return manualMode;
  }

  public boolean autonomousEnabled() {
    return autonomousEnabled;
  }

  public boolean robotEnabled() {
    return enabled;
  }

  public boolean hubEnabled() {
    return hubEnabled;
  }

  public AutoWinnerOverride autoWinnerOverride() {
    return autoWinnerOverride;
  }

  public void setAutoWinnerOverride(AutoWinnerOverride override) {
    autoWinnerOverride = override;
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

  public Trigger shooterHandleBeached() {
    return new Trigger(() -> shooterState == ShooterState.HANDLE_BEACHED).and(normalMode());
  }

  public Trigger canFire() {
    return shooterDisabled().negate()
        .and(normalMode())
        .and(() -> robotEnabled())
        .and(() -> RobotContainer.turret.atGoal())
        .and(() -> shooterState == ShooterState.TARGET_HUB ? true : (RobotContainer.flywheel.atSetpoint()))
        .and(() -> shooterState == ShooterState.TARGET_HUB ? hubEnabled() : true);
  }

  public Trigger shouldFire() {
    return canFire().and(Controls.shootButton);
  }

  public Trigger shouldFireManual() {
    return Controls.shootButton.and(manualMode());
  }

  public Trigger shouldAgitate() {
    return new Trigger(() -> isShooting && !isIntaking);
  }

  public Trigger shouldIntake() {
    return new Trigger(() -> isIntaking);
  }

  public Trigger shouldOuttake() {
    return new Trigger(() -> !isIntaking && !isShooting && outtakeDesired);
  }

  public Trigger shouldStabilize() {
    return Controls.shootButton;
  }
  // double expanderSetpoint = ExpanderPosition.STOWED.getDegrees();
  // if (expanderState == ExpanderState.EXTENDED) {
  // expanderSetpoint = ExpanderPosition.EXTENDED.getDegrees();
  // }

  public Trigger expanderStowed() {
    return new Trigger(() -> expanderState == ExpanderState.STOWED);
  }

  public Trigger expanderExtended() {
    return new Trigger(() -> expanderState == ExpanderState.EXTENDED);
  }

  public Trigger expanderIntaking() {
    return new Trigger(() -> expanderState == ExpanderState.INTAKING);
  }

  public Trigger expanderAgitating() {
    return new Trigger(
        () -> (expanderState == ExpanderState.FULLY_AGITATING
            || expanderState == ExpanderState.AGITATING
            || expanderState == ExpanderState.SMART_AGITATING));
  }
}
