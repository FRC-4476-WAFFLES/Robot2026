// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.sim.SimRobot;

/**
 * Checks the physical robot — the layer that says where the robot actually is,
 * as against where it believes it is.
 *
 * <p>
 * Before this existed the simulation had two ideas of the robot and both of them
 * were odometry, so the robot could never be anywhere the wheels did not say it
 * was: it drove through walls and climbed the bump at whatever speed the wheels
 * turned. What the wheels do is now an intent, and what happens is that intent
 * meeting mass, grip and the field.
 *
 * <p>
 * Driven directly rather than through a booted robot, so what is being checked
 * is the physics and not the plumbing.
 */
public class SimRobotTest {
  private static final ChassisSpeeds STOPPED = new ChassisSpeeds();

  @BeforeEach
  void placeAtOrigin() {
    SimRobot.setPose(new Pose2d(4.0, 4.0, Rotation2d.kZero));
  }

  /** Runs the physics for a while with no slope. */
  private static void drive(ChassisSpeeds wheels, double seconds) {
    for (int i = 0; i < seconds / 0.02; i++) {
      SimRobot.update(wheels, Translation2d.kZero);
    }
  }

  @Test
  void theRobotCarriesMomentum() {
    // The whole point of the layer. Wheels stopping is not the robot stopping.
    drive(new ChassisSpeeds(3.0, 0, 0), 1.5);
    double movingSpeed = SimRobot.getVelocity().getNorm();
    assertTrue(movingSpeed > 2.5, "should be up to speed, was " + movingSpeed);

    // Wheels to a dead stop in one loop. A robot with mass cannot do that.
    SimRobot.update(STOPPED, Translation2d.kZero);
    double afterOneLoop = SimRobot.getVelocity().getNorm();
    System.out.printf("%.2f m/s, wheels stopped, %.2f m/s one loop later%n",
        movingSpeed, afterOneLoop);
    assertTrue(afterOneLoop > 0.5,
        "the robot should still be moving after the wheels stop, was " + afterOneLoop);
  }

  @Test
  void thereIsOnlySoMuchGrip() {
    // Asking for more acceleration than the tyres can deliver makes them slip,
    // which is where hard launches and hard turns lose grip without any of it
    // being a special case.
    drive(new ChassisSpeeds(4.0, 0, 0), 0.1);
    double reached = SimRobot.getVelocity().getNorm();
    double possible = SimRobot.maxTractionAcceleration() * 0.1;
    System.out.printf("asked for 4.00 m/s, reached %.2f m/s in 0.1s (grip allows %.2f)%n",
        reached, possible);
    assertTrue(reached <= possible + 0.05,
        "grip should have limited the launch to " + possible + ", reached " + reached);
  }

  @Test
  void aGentleRequestIsNotLimited() {
    // The limit must only bite when it should, or the robot would feel sluggish
    // everywhere.
    drive(new ChassisSpeeds(0.5, 0, 0), 1.0);
    double reached = SimRobot.getVelocity().getNorm();
    System.out.printf("asked for 0.50 m/s, reached %.2f m/s%n", reached);
    assertTrue(reached > 0.45, "an easy request should be met, reached " + reached);
  }

  @Test
  void hittingSomethingTakesTheSpeedOutOfIt() {
    drive(new ChassisSpeeds(3.0, 0, 0), 1.5);
    double before = SimRobot.getVelocity().getNorm();

    // A wall facing back along the robot's travel.
    SimRobot.collide(SimRobot.getPose(), new Translation2d(-1, 0));
    double after = SimRobot.getVelocity().getNorm();
    System.out.printf("hit a wall at %.2f m/s, left with %.2f m/s%n", before, after);
    assertTrue(after < before * 0.3,
        "a head-on hit should take nearly all the speed, left " + after + " of " + before);
  }

  @Test
  void slidingAlongAWallDoesNotStickToIt() {
    // Only the speed going into the surface is removed. A robot running down a
    // wall keeps running down it, which is what happens and what a driver
    // expects.
    drive(new ChassisSpeeds(0, 3.0, 0), 1.5);
    double before = SimRobot.getVelocity().getNorm();

    SimRobot.collide(SimRobot.getPose(), new Translation2d(-1, 0));
    double after = SimRobot.getVelocity().getNorm();
    System.out.printf("sliding at %.2f m/s along a wall, still %.2f m/s%n", before, after);
    assertTrue(after > before * 0.9,
        "sliding along a wall should keep its speed, kept " + after + " of " + before);
  }

  @Test
  void gravityOnASlopePushesItBack() {
    // Beaching: drive at the bump too gently and the slope wins.
    // With full grip the drivetrain beats any slope the field has, which is
    // right: beaching is a grip failure, not a speed one. On the bump there is
    // less weight on each wheel and a worse surface under them.
    var downhill = new Translation2d(-2.0, 0);
    for (int i = 0; i < 75; i++) {
      SimRobot.update(new ChassisSpeeds(0.3, 0, 0), downhill, 0.15);
    }
    double travelled = SimRobot.getPose().getX() - 4.0;
    System.out.printf("drove at 0.3 m/s up a slope pulling back 2.0 m/s2 on 15%% grip, moved %.2f m%n",
        travelled);
    assertTrue(travelled < 0,
        "the slope should have won and pushed the robot back, moved " + travelled);
  }
}
