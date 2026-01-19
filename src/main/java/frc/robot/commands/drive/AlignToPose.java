// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.utils.lib.WafflesUtilities;

public class AlignToPose extends Command {
  /* Approach Constants */
  public static final double maxAccelerationElevatorUp = 7.0;
  public static final double maxAccelerationElevatorDown = 7.0;
  public static final double defaultMaxVelocity = 4.0;

  public static final double maxThetaAcceleration = 20;
  public static final double maxThetaVelocity = 6;

  // Counteracts the "orbiting" effect caused by always being "attracted" to the target pose
  public static final double onTargetVelocityDeadbandScale = 4;
  public static final double alignmentDecelerationMultiplier = 0.5;

  // Refresh profiles if strafing more than this value
  public static final double strafeResetLimit = 0.1;
  public static final double approachFeedforwardBlendOuter = 0.45; // Distance at which velocity feedforward begins to lose influence
  public static final double approachFeedforwardBlendInner = 0.02; // Distance at which velocity feedforward loses all influence

  /* Controllers */
  private ProfiledPIDController approachPidController = new ProfiledPIDController(2.6, 0, 0.05,
      new Constraints(defaultMaxVelocity, maxAccelerationElevatorDown));
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(7.0, 0, 0.1,
      new Constraints(maxThetaVelocity, maxThetaAcceleration));

  /* Tolerances */
  private double PosMaxError = 0.01; // Meters
  private Rotation2d RotMaxError = Rotation2d.fromDegrees(0.8);

  private double maxVelocity = 4;
  private double maxAcceleration = maxAccelerationElevatorDown;

  /* Data */
  private final Supplier<Pose2d> goalPoseSupplier;
  private Pose2d goalPose = Pose2d.kZero;
  private Pose2d lastGoalPoseRaw = Pose2d.kZero;

  private Trigger endTrigger;
  private double endingDebounce = 0;
  private boolean lockWheelsOnceFinished = true;
  private boolean allianceFlipping = false;
  private boolean goalPoseChanged = true;

  private double lastMaxAcceleration = maxAccelerationElevatorDown; // Cache acceleration to reduce allocations

  private final Timer alignmentTimer = new Timer();

  /** 
   * Command that drives the robot to align with a desired pose
   * @param targetPoseSupplier The goal pose for the robot to align to as a supplier
   */
  public AlignToPose(Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(RobotContainer.driveSubsystem);

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    goalPoseSupplier = targetPoseSupplier;

    endTrigger = new Trigger(() -> isAtGoal())
        .debounce(endingDebounce);
  }

  /** 
   * Command that drives the robot to align with a desired pose
   * @param targetPose The goal pose for the robot to align to
   */
  public AlignToPose(Pose2d targetPose) {
    // Convenience so you don't have to write a supplier
    this(() -> targetPose);
  }

  /**
   * Sets if wheels should be locked once alignment finishes
   * @param lockWheelsOnceFinished A boolean
   */
  public AlignToPose withShouldLockWheels(boolean lockWheelsOnceFinished) {
    this.lockWheelsOnceFinished = lockWheelsOnceFinished;

    return this;
  }

  /**
   * Sets if poses passed in should be flipped automatically if on the red alliance
   * @param lockWheelsOnceFinished A boolean
   */
  public AlignToPose withAllianceFlipping(boolean shouldFlipAlliance) {
    this.allianceFlipping = shouldFlipAlliance;
    lastGoalPoseRaw = Pose2d.kZero; // Invalidate cache in edge case

    return this;
  }

  /**
   * Sets the time to wait once at the target before ending the command (defaults to zero) 
   * @param endingDebounce A time in seconds
   */
  public AlignToPose withEndingDebounce(double endingDebounce) {
    this.endingDebounce = endingDebounce;

    endTrigger = new Trigger(() -> isAtGoal())
        .debounce(endingDebounce);

    return this;
  }

  public AlignToPose withMaxVelocity(double maxVelocity) {
    this.maxVelocity = maxVelocity;
    updateConstraints(true);

    return this;
  }

  /**
   * Sets the max position tolerance for being on target (default 0.01 meters)
   * @param tolerance a value in meters
   */
  public AlignToPose withPositionTolerance(double tolerance) {
    PosMaxError = tolerance;
    approachPidController.setTolerance(PosMaxError, 0.04); // Keep default velocity tolerance

    return this;
  }

  /**
   * Sets the max rotation tolerance for being on target (default 0.5 degrees)
   * @param tolerance a rotation
   */
  public AlignToPose withThetaTolerance(Rotation2d tolerance) {
    RotMaxError = tolerance;
    thetaPidController.setTolerance(RotMaxError.getRadians(), Math.toRadians(1.0)); // Keep default velocity tolerance

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("AlignmentMetrics/Is Aligning", true);

    // Update goal pose once from supplier
    updateGoalPose();

    // Start the alignment timer
    alignmentTimer.reset();
    alignmentTimer.start();

    var currentPose = RobotContainer.driveSubsystem.getPose();
    ChassisSpeeds currentSpeeds = RobotContainer.driveSubsystem.getFieldVelocity();

    // Set tolerances (both position AND velocity for proper atGoal() behavior)
    thetaPidController.setTolerance(RotMaxError.getRadians(), Math.toRadians(1.0)); // 5 deg/s velocity tolerance
    approachPidController.setTolerance(PosMaxError, 0.08); // 8 cm/s velocity tolerance

    // Reset theta controller
    thetaPidController.reset(currentPose.getRotation().getRadians(),
        RobotContainer.driveSubsystem.getFieldVelocity().omegaRadiansPerSecond);

    // Reset approach controller
    double distanceToTarget = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentSpeeds,
        WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(), goalPose.getTranslation()));

    approachPidController.reset(distanceToTarget,
        Math.min(
            0.0,
            -velocityTowardsTarget.getX() // Approach velocity is negative since we PID towards zero
        ));

    // Telemetry
    Logger.recordOutput("AlignmentMetrics/Approach Velocity", velocityTowardsTarget.getX());
    Logger.recordOutput("AlignmentMetrics/Strafe Velocity", velocityTowardsTarget.getY());
    Logger.recordOutput("AlignmentMetrics/Goal Pose", goalPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update goal pose once from supplier
    updateGoalPose();

    updateConstraints(false);

    // Update max acceleration based on elevator height (optimized to reduce allocations)
    double maxAcceleration = MathUtil.interpolate(maxAccelerationElevatorDown, maxAccelerationElevatorUp,
        RobotContainer.superstructure.elevator.getElevatorExtendedPercent());

    // Only update constraints if acceleration changed (reduces allocations)
    if (Math.abs(maxAcceleration - lastMaxAcceleration) > 0.01) { // 0.1 m/s² threshold
      approachPidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
      lastMaxAcceleration = maxAcceleration;
    }

    Logger.recordOutput("AlignmentMetrics/Max Acceleration", maxAcceleration);

    // Get current conditions
    Pose2d currentPose = RobotContainer.driveSubsystem.getPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Rotation2d angleToTarget = WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(),
        goalPose.getTranslation());
    ChassisSpeeds currentSpeeds = RobotContainer.driveSubsystem.getFieldVelocity();
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentSpeeds, angleToTarget); // This is in target space

    if ((distanceToTarget > 0.3 && Math.abs(velocityTowardsTarget.getY()) > 0.5) ||
        goalPoseChanged) {
      // Approach velocity is negative since we PID towards zero
      approachPidController.reset(distanceToTarget,
          Math.min(
              0.0,
              -velocityTowardsTarget.getX()
          ));
    }

    // Drive to pose with PID

    // Blend between feedforward and feedback control
    double approachVelocityFeedforwardBlend = MathUtil.clamp(
        WafflesUtilities.InvLerp(approachFeedforwardBlendInner, approachFeedforwardBlendOuter, distanceToTarget),
        0, 1);
    approachVelocityFeedforwardBlend = WafflesUtilities.QuadraticEaseOut(approachVelocityFeedforwardBlend); // Square blending factor for smoothness
    // feedforwardBlendPublisher.set(approachVelocityFeedforwardBlend);
    Logger.recordOutput("AlignmentMetrics/FF Blend", approachVelocityFeedforwardBlend);

    // Calculate velocity feedforward
    double approachVelocityFeedback = -approachPidController.calculate(distanceToTarget, 0);
    double approachVelocityFeedForward = Math.max(-approachPidController.getSetpoint().velocity, 0.3);

    // Calculate target velocities
    double targetApproachVelocity = (approachVelocityFeedForward * approachVelocityFeedforwardBlend) +
        (approachVelocityFeedback * (1 - approachVelocityFeedforwardBlend));

    // Deadband target velocity
    if (distanceToTarget < PosMaxError) {
      targetApproachVelocity = 0;
    }

    // approachOutputPublisher.set(targetApproachVelocity);
    Logger.recordOutput("AlignmentMetrics/Approach Output", targetApproachVelocity);

    // Calculate final field velocity
    Translation2d targetFieldVelocity = new Translation2d(targetApproachVelocity, angleToTarget);

    // Calculate rotation output
    double targetThetaVelocity = thetaPidController.calculate(currentPose.getRotation().getRadians(),
        goalPose.getRotation().getRadians());

    // Deadband target velocity
    if (Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees()) < RotMaxError.getDegrees()) {
      targetThetaVelocity = 0;
    }

    // Publish telemetry
    Logger.recordOutput("AlignmentMetrics/Theta Output", targetThetaVelocity);
    Logger.recordOutput("AlignmentMetrics/Approach Setpoint Position", approachPidController.getSetpoint().position);
    Logger.recordOutput("AlignmentMetrics/Approach Setpoint Velocity", approachPidController.getSetpoint().velocity);

    // distancePublisher.set(distanceToTarget);
    Logger.recordOutput("AlignmentMetrics/Approach Distance", distanceToTarget);

    Logger.recordOutput("AlignmentMetrics/Approach Velocity", velocityTowardsTarget.getX());
    Logger.recordOutput("AlignmentMetrics/Strafe Velocity", velocityTowardsTarget.getY());
    Logger.recordOutput("AlignmentMetrics/Goal Pose", goalPose);

    // Apply chosen velocity
    applyFieldVelocity(targetFieldVelocity, targetThetaVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("AlignmentMetrics/Is Aligning", false);

    // Stop the timer and publish the final alignment time
    alignmentTimer.stop();
    double finalAlignmentTime = alignmentTimer.get();

    Logger.recordOutput("AlignmentMetrics/Alignment Time", finalAlignmentTime);

    if (lockWheelsOnceFinished) {
      // Stop drivetrain
      applyFieldVelocity(Translation2d.kZero, 0);
    }
  }

  /**
   * If current pose is within a certain range of target (both position and velocity)
   */
  public boolean isAtGoal() {
    // Use controller's built-in atGoal() which considers both position AND velocity
    return approachPidController.atGoal() && thetaPidController.atGoal();
  }

  private void updateConstraints(boolean forceRefresh) {
    // Update max acceleration based on elevator height (optimized to reduce allocations)
    maxAcceleration = MathUtil.interpolate(maxAccelerationElevatorDown, maxAccelerationElevatorUp,
        RobotContainer.superstructure.elevator.getElevatorExtendedPercent());

    // Only update constraints if acceleration changed significantly (reduces allocations)
    if (Math.abs(maxAcceleration - lastMaxAcceleration) > 0.1 || forceRefresh) { // 0.1 m/s² threshold
      approachPidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
      lastMaxAcceleration = maxAcceleration;
    }

    Logger.recordOutput("AlignmentMetrics/Max Acceleration", maxAcceleration);
  }

  /**
   * Helper method to apply a chosen field velocity to the drivetrain
   */
  private void applyFieldVelocity(Translation2d targetVelocity, double targetThetaVelocity) {
    RobotContainer.driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        targetVelocity.getX(),
        targetVelocity.getY(),
        targetThetaVelocity
    ), RobotContainer.driveSubsystem.getRotation()));
  }

  private void updateGoalPose() {
    var rawPose = goalPoseSupplier.get();
    if (lastGoalPoseRaw.relativeTo(rawPose).getTranslation().getNorm() < 0.01) {
      goalPoseChanged = false;
      return;
    }

    if (allianceFlipping) {
      goalPose = WafflesUtilities.FlipIfRedAlliance(rawPose);
    } else {
      goalPose = rawPose;
    }

    goalPoseChanged = true;
    lastGoalPoseRaw = rawPose;
  }

  /**
   * Calculates current velocity towards the target pose 
   * X+ is forward
   */
  private static Translation2d getVelocityTowardsTarget(ChassisSpeeds currentSpeeds, Rotation2d angleBetweenPoses) {
    Translation2d fieldVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    return fieldVelocity.rotateBy(angleBetweenPoses.unaryMinus());
  }

  @Override
  public boolean isFinished() {
    return endTrigger.getAsBoolean();
  }
}