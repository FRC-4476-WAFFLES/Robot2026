// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class WafflesUtilities {
  /**
   * Private constructor to prevent instantiation of utility class
   */
  private WafflesUtilities() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /**
   * For for some reason there's no lerp in Java
   */
  public static double Lerp(double num1, double num2, double t) {
    return num1 + ((num2 - num1) * t);
  }

  /**
   * Again, for some reason there's no inverse lerp in Java
   */
  public static double InvLerp(double num1, double num2, double t) {
    return (t - num1) / (num2 - num1);
  }

  public static double translationDotProduct(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /**
   * Converts an timestamp in the timebase of {@link #getCurrentTimeSeconds()}
   * to the timebase reported by the FPGA
   *
   * @param timestampCurrentTime The current timestamp in seconds
   * @return The equivalent Timer.getFPGATimestamp() timestamp in seconds
   */
  public static double currentTimeToFPGA(double timestampCurrentTime) {
    return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + timestampCurrentTime;
  }

  public static Transform2d Transform3dTo2d(Transform3d input) {
    return new Transform2d(input.getTranslation().toTranslation2d(), input.getRotation().toRotation2d());
  }

  /**
   * Takes a pose and flips it to the other side of the field if the robot is on the red alliance.
   * @param pose The input Pose2d
   * @return The output Pose2d 
   */
  public static Pose2d FlipIfRedAlliance(Pose2d pose) {
    if (IsRedAlliance()) {
      return FlippingUtil.flipFieldPose(pose);
    }

    return pose;
  }

  /**
  * Takes a translation and flips it to the other side of the field if the robot is on the red alliance.
  * @param position The input Translation2d
  * @return The output Translation2d
  */
  public static Translation2d FlipIfRedAlliance(Translation2d position) {
    if (IsRedAlliance()) {
      return FlippingUtil.flipFieldPosition(position);
    }

    return position;
  }

  /**
   * Takes an angle and flips it to the other side of the field if the robot is on the red alliance.
   * @param angle The input angle (degrees)
   * @return The output angle 
   */
  public static Rotation2d FlipIfRedAlliance(Rotation2d angle) {
    if (IsRedAlliance()) {
      return FlippingUtil.flipFieldRotation(angle);
    }

    return angle;
  }

  /**
   * Takes an x coordinate and flips it to the other side of the field if the robot is on the red alliance.
   * @param x The x coordinate (meters)
   * @return The output coordinate 
   */
  public static double FlipXIfRedAlliance(double x) {
    if (IsRedAlliance()) {
      return FlippingUtil.fieldSizeX - x;
    }

    return x;
  }

  /**
  * Takes an y coordinate and flips it to the other side of the field if the robot is on the red alliance.
  * @param y The y coordinate (meters)
  * @return The output coordinate 
  */
  public static double FlipYIfRedAlliance(double y) {
    if (IsRedAlliance()) {
      switch (FlippingUtil.symmetryType) {
        case kMirrored:
          return y;
        case kRotational:
          return FlippingUtil.fieldSizeY - y;
      }
    }

    return y;
  }

  public static boolean IsRedAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns the angle from p1 to p2 from the x axis with positive being up
  */
  public static Rotation2d AngleBetweenPoints(Translation2d p1, Translation2d p2) {
    // Trig implementation
    // return Rotation2d.fromRadians(Math.atan2(p2.getY() - p1.getY(), p2.getX() -
    // p1.getX()));

    // WPILIB implementation
    return p2.minus(p1).getAngle();
  }

  /*
   * Eases exponentially out over a range of 0-1
   * Inputs must be clamped
   */
  public static double QuadraticEaseOut(double v) {
    return 1 - Math.pow(1 - v, 2);
  }

  /**
   * Returns whether our alliance's hub is currently active for scoring.
   *
   * During ALLIANCE SHIFT (2:10 to 0:55 on teleop timer), hubs alternate
   * being active in windows. The game specific message ('R' or 'B') indicates
   * which alliance is inactive first.
   *
   * Before 2:10 and during endgame (0:30 and below), both hubs are active.
   */
  public static boolean isHubActive() {
    double matchTime = DriverStation.getMatchTime();

    if (matchTime < 0 || matchTime > 130 || matchTime <= 30) {
      return true;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return true;
    }

    boolean weGoFirstInactive = (gameData.charAt(0) == 'R') == IsRedAlliance();

    boolean inOddWindow = (matchTime > 105 && matchTime <= 130)
                       || (matchTime > 55 && matchTime <= 80);

    if (weGoFirstInactive) {
      return !inOddWindow;
    } else {
      return inOddWindow;
    }
  }

  /**
   * Returns the angle for the driver's forward direction, depending on alliance
   * @return a rotation2d representing driver forward in field space
   */
  public static Rotation2d getDriverForwardAngle() {
    if (IsRedAlliance()) {
      return Rotation2d.k180deg;
    }

    return Rotation2d.kZero;
  }
}