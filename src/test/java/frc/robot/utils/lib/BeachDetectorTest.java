// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The numbers here are not invented. They come from recorded autonomous
 * periods, and each test is a situation the detector has to get right on a real
 * field rather than a shape that happens to exercise the code.
 */
public class BeachDetectorTest {
  private static final double DT = 0.02;
  private BeachDetector detector;
  private double clock;
  private double travelled;

  @BeforeEach
  void setUp() {
    detector = new BeachDetector();
    clock = 0;
    travelled = 0;
  }

  /** Runs the detector for a while at a fixed speed and a fixed rate of progress. */
  private void feed(double seconds, double wheelSpeed, double actualSpeed, double commanded) {
    for (double t = 0; t < seconds; t += DT) {
      travelled += actualSpeed * DT;
      clock += DT;
      detector.update(clock, new Translation2d(travelled, 0), wheelSpeed, commanded, DT);
    }
  }

  @Test
  void drivingNormallyIsNotBeaching() {
    // The wheels turn and the robot goes with them.
    feed(3.0, 1.6, 1.6, 1.9);
    assertFalse(detector.isBeached(), "ordinary driving must not read as stuck");
  }

  @Test
  void sittingStillOnPurposeIsNotBeaching() {
    // Autonomous spends 8.2 s per run stopped and shooting. Nothing is wrong
    // with the robot and nothing is being asked of the drivetrain.
    feed(3.0, 0.0, 0.0, 0.0);
    assertFalse(detector.isBeached(), "standing still while not asked to drive is fine");
  }

  @Test
  void theHoustonQ4Beaching() {
    // Measured: the wheels reported a steady 1.4 m/s while the robot's actual
    // position moved 3 cm per second, and it was still being commanded to
    // 1.4 m/s. It stayed like that for 13.4 s and nothing noticed.
    feed(2.0, 1.42, 1.42, 1.40);
    assertFalse(detector.isBeached(), "should be clean before it rides up");

    feed(2.0, 1.42, 0.03, 1.40);
    assertTrue(detector.isBeached(),
        "the q4 beaching must be caught; ground ratio was " + detector.getGroundRatio());
    assertTrue(detector.getGroundRatio() < 0.1,
        "ratio should show the robot going nowhere, was " + detector.getGroundRatio());
  }

  @Test
  void itRecoversWhenTheRobotGetsGoingAgain() {
    feed(2.0, 1.42, 0.03, 1.40);
    assertTrue(detector.isBeached(), "precondition: it should be stuck here");
    feed(1.5, 1.42, 1.42, 1.40);
    assertFalse(detector.isBeached(), "once it is moving again the flag must clear");
  }

  @Test
  void aBumpCrossingIsNotBeaching() {
    // The one that matters. Crossing a bump costs about a third of wheel
    // travel, and it is survived by carrying on -- reacting to it as a beaching
    // would abandon a perfectly good crossing. Measured slip there leaves the
    // robot covering roughly two thirds of what the wheels turn, far above the
    // 0.30 the detector demands.
    feed(3.0, 1.5, 1.0, 1.6);
    assertFalse(detector.isBeached(),
        "a bump crossing must not read as beaching, ratio was " + detector.getGroundRatio());
  }

  @Test
  void oneBadPoseDoesNotTripIt() {
    // A single vision correction can move the estimate. That is not evidence of
    // anything, and the confirmation window exists to ignore it.
    feed(2.0, 1.5, 1.5, 1.6);
    clock += DT;
    detector.update(clock, new Translation2d(0, 0), 1.5, 1.6, DT);
    assertFalse(detector.isBeached(), "a single outlying pose must not be enough");
  }

  @Test
  void aLoopGapDoesNotIntegrateAHole() {
    feed(2.0, 1.5, 1.5, 1.6);
    // A 300 ms stall. Anything measured across it is meaningless.
    clock += 0.30;
    detector.update(clock, new Translation2d(travelled + 3.0, 0), 1.5, 1.6, 0.30);
    assertFalse(detector.isBeached(), "a gap in the loop must clear the history, not trip it");
  }
}
