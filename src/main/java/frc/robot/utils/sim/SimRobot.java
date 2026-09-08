// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.data.Constants.CodeConstants;

/**
 * Where the robot actually is, as against where it believes it is.
 *
 * <p>
 * The simulation had two ideas of the robot and both of them were odometry. The
 * pose the code uses is integrated from wheel motion, and the "truth" pose the
 * cameras looked at was the same integration with corrections bolted on, so the
 * robot could never be anywhere the wheels did not say it was. Drive into a wall
 * and it went through; drive at the bump and it climbed at whatever speed the
 * wheels turned.
 *
 * <p>
 * This is the third layer. It carries momentum, it can only push as hard as the
 * tyres grip, and it is stopped by the field. What the wheels do is an
 * <i>intent</i>; what happens to the robot is the result of that intent meeting
 * physics. Odometry keeps integrating the wheels either way, which is precisely
 * why it drifts — and drifts in the same circumstances a real one does.
 *
 * <p>
 * The values below are honest about where they came from. Mass and moment of
 * inertia are from the robot's real dimensions and a weighed estimate; the
 * coefficient of friction is a textbook figure for a wheel on carpet and is the
 * one number here worth measuring before trusting a result that turns on it.
 */
public final class SimRobot {
  /** Kilograms, a competition robot with bumpers and battery. */
  private static final double MASS = 55.0;
  /**
   * Wheel on carpet. The number that decides how hard the robot can accelerate
   * before the wheels break loose, and the one worth measuring.
   */
  private static final double FRICTION_COEFFICIENT = 1.1;
  private static final double GRAVITY = 9.81;
  /** How hard the robot can push before the tyres let go, in m/s². */
  private static final double MAX_TRACTION_ACCELERATION = FRICTION_COEFFICIENT * GRAVITY;
  /** Rotational equivalent, in rad/s². Scaled by the robot's radius of gyration. */
  private static final double MAX_ANGULAR_ACCELERATION = 30.0;
  /** How much speed is lost in a collision rather than returned as a bounce. */
  private static final double RESTITUTION = 0.1;

  private static Pose2d pose = Pose2d.kZero;
  private static Translation2d velocity = Translation2d.kZero;
  private static double angularVelocity = 0;

  private SimRobot() {}

  /** Where the robot actually is. */
  public static Pose2d getPose() {
    return pose;
  }

  /** How fast it is actually moving, field relative, in m/s. */
  public static Translation2d getVelocity() {
    return velocity;
  }

  /** Places the robot, discarding whatever it was doing. */
  public static void setPose(Pose2d newPose) {
    pose = newPose;
    velocity = Translation2d.kZero;
    angularVelocity = 0;
  }

  /**
   * Advances the robot by one loop.
   *
   * @param wheelSpeeds what the wheels are trying to do to the robot, field
   *     relative. Not what the robot will do — that is what this works out.
   * @param slopeAcceleration gravity along whatever surface the robot is on,
   *     field relative, in m/s²
   */
  public static void update(ChassisSpeeds wheelSpeeds, Translation2d slopeAcceleration) {
    update(wheelSpeeds, slopeAcceleration, 1.0);
  }

  /**
   * Advances the robot by one loop, on a surface with less grip than carpet.
   *
   * @param tractionScale how much of the usual grip is available. The bump is
   *     the case that matters: a robot on it has less weight on each wheel and a
   *     worse surface under them, and if what is left cannot beat gravity along
   *     the slope, the robot slides back down however hard it drives. That is
   *     beaching, and it falls out of the physics rather than being a special
   *     case.
   */
  public static void update(ChassisSpeeds wheelSpeeds, Translation2d slopeAcceleration,
      double tractionScale) {
    double dt = CodeConstants.PERIODIC_LOOP_TIME;

    // What the wheels are asking the robot to do, and what it would take to
    // achieve it this loop.
    Translation2d wanted = new Translation2d(
        wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond);
    Translation2d needed = wanted.minus(velocity).div(dt);

    // The tyres can only pull so hard. Ask for more and they slip, which is
    // where hard acceleration and hard turns lose grip without being special
    // cases.
    double available = MAX_TRACTION_ACCELERATION * MathUtil.clamp(tractionScale, 0, 1);
    double demanded = needed.getNorm();
    Translation2d applied = demanded > available
        ? needed.times(available / demanded)
        : needed;

    velocity = velocity.plus(applied.plus(slopeAcceleration).times(dt));

    double wantedOmega = wheelSpeeds.omegaRadiansPerSecond;
    double neededOmega = (wantedOmega - angularVelocity) / dt;
    angularVelocity += MathUtil.clamp(neededOmega,
        -MAX_ANGULAR_ACCELERATION, MAX_ANGULAR_ACCELERATION) * dt;

    pose = new Pose2d(
        pose.getTranslation().plus(velocity.times(dt)),
        pose.getRotation().plus(Rotation2d.fromRadians(angularVelocity * dt)));

    Logger.recordOutput("SimRobot/Pose", pose);
    Logger.recordOutput("SimRobot/Speed", velocity.getNorm());
    Logger.recordOutput("SimRobot/Slipping", demanded > available);
    Logger.recordOutput("SimRobot/Traction Demand", demanded / Math.max(0.01, available));
    Logger.recordOutput("SimRobot/Traction Available", available);
  }

  /**
   * Stops the robot dead against a surface it has hit.
   *
   * <p>
   * Only the part of the velocity going into the surface is removed, so a robot
   * sliding along a wall keeps sliding rather than sticking to it. A little of
   * it comes back as a bounce, because a robot at speed hitting a wall does not
   * simply stop.
   *
   * @param normal unit vector pointing away from the surface
   */
  public static void collide(Pose2d correctedPose, Translation2d normal) {
    pose = correctedPose;
    double into = velocity.getX() * normal.getX() + velocity.getY() * normal.getY();
    if (into < 0) {
      velocity = velocity.minus(normal.times(into * (1 + RESTITUTION)));
    }
  }

  /** The most the robot can accelerate before the wheels break loose, in m/s². */
  public static double maxTractionAcceleration() {
    return MAX_TRACTION_ACCELERATION;
  }

  /** Robot mass in kilograms, for anything that needs to reason about force. */
  public static double mass() {
    return MASS;
  }
}
