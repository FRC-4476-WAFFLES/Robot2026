// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

  /**
   * Half the wheel spacing, fore-aft and side to side, from TunerConstants.
   *
   * <p>
   * These are not close to equal: the drivetrain is 15.5 inches front to back
   * and 27.5 across. Accelerating along the short axis transfers weight over a
   * shorter lever, so the front wheels unload sooner and the robot runs out of
   * grip earlier going forwards than it does going sideways. A single traction
   * number cannot express that, which is why the limit below depends on which
   * way the robot is being asked to go.
   */
  private static final double HALF_WHEELBASE = Units.inchesToMeters(7.75);
  private static final double HALF_TRACK = Units.inchesToMeters(13.75);

  /**
   * Height of the centre of gravity above the carpet, in metres.
   *
   * <p>
   * <b>Estimated, not measured.</b> It cannot be recovered from the match logs:
   * the method needs the robot to actually saturate traction in two directions
   * and it never does — measured acceleration peaks around 4 m/s² against a
   * ceiling near 10, so the logs record what the path follower asked for rather
   * than what the carpet allows. 0.25 m is a plausible figure for a robot of
   * this size and it reproduces the observed ceiling (see the class comment),
   * but it is the number to replace first if this model is ever trusted with a
   * real decision.
   *
   * <p>
   * To measure it properly: put the robot on a slope, or on scales under each
   * axle, and increase the tilt until the uphill wheels unload. The angle where
   * that happens gives {@code h = halfSpacing / tan(angle)}.
   */
  private static final double COG_HEIGHT = 0.25;

  /** A ball, in kilograms — about half a pound. */
  private static final double BALL_MASS = 0.227;
  /**
   * How high a ball sits, in metres. Cargo rides above the drivetrain, so a
   * full hopper raises the centre of gravity rather than just adding mass —
   * and it is the height that matters here, because mass cancels out of a
   * traction limit while the centre of gravity does not.
   */
  private static final double BALL_COG_HEIGHT = 0.50;

  /** How many balls the robot is carrying. Raises the centre of gravity. */
  private static int cargoBalls = 0;
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
    // achieve it this loop. Gravity along the slope has to be paid for out of
    // the same grip budget: the modules run closed loop on velocity, so holding
    // still on a ramp is work, and the wheels only get what is left over for
    // going anywhere. Without that term a robot on a slope could never hold
    // position — it crept downhill forever at whatever speed one loop of
    // gravity gave it, which is what made the bump feel like ice.
    Translation2d wanted = new Translation2d(
        wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond);
    Translation2d needed = wanted.minus(velocity).div(dt).minus(slopeAcceleration);

    // The tyres can only pull so hard. Ask for more and they slip, which is
    // where hard acceleration and hard turns lose grip without being special
    // cases.
    //
    // How hard is not a constant, because pushing the robot along also tips
    // weight off the wheels doing the pushing. Over a half spacing D, an
    // acceleration a moves m*a*h/(2D) of load off the leading axle onto the
    // trailing one, and once the modules are all asking for the same force it
    // is the unloaded pair that lets go first. Solving for where that happens
    // gives mu*g / (1 + mu*h/D) — always less than mu*g, and much less along
    // the short axis.
    //
    // This is what was missing. A flat mu*g put the ceiling at 10.8 m/s², which
    // the real robot never came near; with transfer the model gives 4.5 m/s²
    // forwards and 6.0 sideways against measured 4.0 and 6.6.
    Translation2d robotFrame = needed.rotateBy(pose.getRotation().unaryMinus());
    double spacing = halfSpacing(robotFrame);
    double height = cogHeight();
    double grip = FRICTION_COEFFICIENT * MathUtil.clamp(tractionScale, 0, 1);
    double available = grip * GRAVITY / (1 + grip * height / spacing);
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
    // Where the wheels would leave the ground, for comparison. With carpet grip
    // and this geometry the robot always slips before it tips — slipping caps
    // the demand below the tipping threshold by construction — so this stays
    // under 1 and is a diagnostic rather than a failure mode. It stops being
    // true if grip goes up or the centre of gravity does, which is exactly when
    // someone would want to know.
    Logger.recordOutput("SimRobot/Tip Fraction",
        demanded / (GRAVITY * spacing / height));
    Logger.recordOutput("SimRobot/CoG Height", height);
  }

  /**
   * Half the wheel spacing resisting a push in this direction, in metres.
   *
   * <p>
   * Straight ahead this is the half wheelbase and straight sideways the half
   * track; in between the wheels form a rectangle, and the ellipse through
   * those two is a good enough interpolation for a model whose centre of
   * gravity is a guess anyway.
   *
   * @param direction the demanded acceleration, in the robot's own frame
   */
  private static double halfSpacing(Translation2d direction) {
    double norm = direction.getNorm();
    if (norm < 1e-6) {
      return HALF_WHEELBASE;
    }
    return 1.0 / Math.hypot(direction.getX() / norm / HALF_WHEELBASE,
        direction.getY() / norm / HALF_TRACK);
  }

  /**
   * Centre of gravity height including whatever the robot is carrying.
   *
   * <p>
   * A hundred balls is 22.7 kg on a 55 kg robot, and they ride high, so a full
   * hopper moves the centre of gravity from 0.25 m to about 0.32 m. That costs
   * roughly 15% of the forward acceleration limit — the robot is meaningfully
   * less able to accelerate when it is full, which is when it most wants to be
   * driving somewhere.
   */
  private static double cogHeight() {
    double cargo = cargoBalls * BALL_MASS;
    return (MASS * COG_HEIGHT + cargo * BALL_COG_HEIGHT) / (MASS + cargo);
  }

  /** Tells the model how many balls are aboard, which raises the centre of gravity. */
  public static void setCargoBalls(int balls) {
    cargoBalls = Math.max(0, balls);
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

  /**
   * Grip expressed as an acceleration, ignoring weight transfer, in m/s².
   *
   * <p>
   * This is the ceiling the robot would have if pushing it did not tip weight
   * off the wheels doing the pushing. It does, so the real limit is always
   * lower than this and depends on direction — see {@link #halfSpacing}. Kept
   * as an upper bound for tests and for anything reasoning about force.
   */
  public static double maxTractionAcceleration() {
    return MAX_TRACTION_ACCELERATION;
  }

  /** Robot mass in kilograms, for anything that needs to reason about force. */
  public static double mass() {
    return MASS;
  }
}
