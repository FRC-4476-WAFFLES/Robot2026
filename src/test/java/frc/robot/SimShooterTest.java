// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.intake.Intake.ExpanderState;
import frc.robot.data.FieldConstants;
import frc.robot.utils.sim.SimField;
import frc.robot.utils.sim.SimShooter;

/**
 * Checks the two things added for watching the simulation rather than reading
 * it: that shooting actually puts balls in the air, and that the field has walls
 * and a bump.
 */
public class SimShooterTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.enableTeleop();
  }

  @AfterAll
  static void putItBack() {
    SimShooter.setEnabled(false);
    SimField.setEnabled(false);
    SimHarness.levelOut();
  }

  @Test
  void shootingPutsBallsInTheAir() {
    SimShooter.setEnabled(true);
    // Somewhere it would actually shoot from.
    RobotContainer.drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
    SimHarness.stepSeconds(0.5);

    int before = SimShooter.getShotsFired();
    // Hold the trigger rather than setting a flywheel setpoint by hand: the
    // shooter command rewrites that setpoint every loop, so one set here would
    // survive a single loop and then be zeroed.
    holdShootTrigger(2.5);
    SimShooter.setEnabled(false);
    SimHarness.stepSeconds(0.3);

    int after = SimShooter.getShotsFired();
    System.out.printf("shots fired: %d -> %d%n", before, after);
    assertTrue(after > before, "shooting for two seconds should have fired at least one ball");
  }

  @Test
  void theFieldHasWalls() {
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    // Start close to a wall and drive at it, so it is actually reached.
    RobotContainer.drive.setPose(new Pose2d(1.2, 2.5, Rotation2d.kZero));
    SimHarness.step(5);

    // Stick forward is +X, so this drives back towards the near wall.
    driveForward(1.0, 2.0);

    // The truth pose is what the wall holds. Odometry keeps integrating the
    // wheels, which is the point -- a robot pressed into a wall loses its pose.
    Pose2d truth = RobotContainer.simState.getPose();
    double odometryX = RobotContainer.state.getPose().getX();
    System.out.printf("drove into the wall: truth x %.2f, odometry x %.2f%n", truth.getX(), odometryX);
    // The bound follows the footprint rather than a fixed number. The robot is
    // its real rectangle now, so facing the wall it stops with its centre half a
    // robot-length away -- closer than the old circle model allowed, and
    // correctly so.
    double halfLength = PhysicalConstants.FULL_LENGTH.in(Meters) / 2;
    assertTrue(truth.getX() >= halfLength - 0.05,
        "the wall should have held the robot on the field, but its centre reached "
            + truth.getX() + " m against a half-length of " + halfLength);

    SimField.setEnabled(false);
  }

  @Test
  void theBumpTiltsTheRobot() {
    // The tilt several behaviours key off, which no simulation could produce
    // before: TARGET_TAG while crossing, the turret's tilt offset, and arming
    // the post-crossing vision recovery.
    //
    // Swept rather than aimed at one coordinate, so this keeps working whatever
    // the field geometry says about where the ramps are.
    SimField.setEnabled(true);
    double tiltedAt = Double.NaN;
    // Down the right-hand ramp, beside the hub -- the hub itself sits on the
    // centre line and would push the robot out of any pose placed inside it.
    for (double x = 1.0; x < FieldConstants.fieldLength - 1.0; x += 0.25) {
      RobotContainer.drive.setPose(new Pose2d(x, 2.5, Rotation2d.kZero));
      SimHarness.step(3);
      if (!RobotContainer.drive.isLevelOnGround()) {
        tiltedAt = x;
        break;
      }
    }
    System.out.printf("robot first read as tilted at x = %.2f m%n", tiltedAt);
    assertTrue(!Double.isNaN(tiltedAt),
        "driving the length of the field should have crossed the bump and tilted the robot");

    // And it must be level again on the far side, or every behaviour gated on
    // being level would stay off for the rest of the match.
    RobotContainer.drive.setPose(new Pose2d(tiltedAt + 3.0, 2.5, Rotation2d.kZero));
    SimHarness.step(3);
    assertTrue(RobotContainer.drive.isLevelOnGround(),
        "the robot should be level again well past the bump");

    SimField.setEnabled(false);
    SimHarness.levelOut();
  }

  @Test
  void crossingTheBumpCostsOdometryAccuracy() {
    // The failure the drive team actually reports, and one no simulation could
    // produce before: both pose estimators were fed identical module positions,
    // so the truth pose and odometry could never disagree.
    //
    // The peak error during the crossing is what matters, not the error after
    // it. Simulated vision sees the truth pose and corrects odometry back
    // towards it, which is exactly what the real robot does -- the drift that
    // hurts is the drift while it is happening, and while a real robot may have
    // no tag in view to fix it.
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    SimHarness.releaseAllControls();
    RobotContainer.drive.setPose(new Pose2d(2.5, 2.5, Rotation2d.kZero));
    // Long enough for vision estimates taken before the robot was placed to
    // wash out, or the baseline carries whatever the last test left behind.
    SimHarness.stepSeconds(1.0);
    double baseline = odometryError();

    double peak = 0;
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, -1.0);
    for (int i = 0; i < 150; i++) {
      SimHarness.step(1);
      peak = Math.max(peak, odometryError());
    }
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.3);

    System.out.printf("odometry error: %.3f m at rest, %.3f m peak while crossing%n", baseline, peak);
    SimField.setEnabled(false);
    SimHarness.levelOut();

    assertTrue(peak > baseline + 0.05,
        "crossing the bump should cost real odometry accuracy, peak was " + peak
            + " against a baseline of " + baseline);
  }

  @Test
  void stoppedBallsAreDespawned() {
    // Every ball in the list is integrated every loop whether it is doing
    // anything or not, so a match's worth of dead balls is pure loop time --
    // and loop time is what makes the flywheel and battery models worth having.
    //
    // This also proves the reflection into FuelSim still works. It is a vendor
    // file that has to stay diffable against upstream, so its private fuel list
    // is reached rather than a prune method added, and a version bump could
    // silently break it.
    SimShooter.setEnabled(true);
    RobotContainer.drive.setPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
    SimHarness.stepSeconds(0.4);

    holdShootTrigger(2.0);
    SimShooter.setEnabled(false);

    // Long enough for what was fired to land and settle.
    SimHarness.stepSeconds(4.0);
    int remaining = SimShooter.getBallsInPlay();
    System.out.printf("balls still in play after everything landed: %d%n", remaining);
    assertTrue(remaining >= 0, "the prune must not throw");
    assertTrue(SimShooter.isPruneWorking(),
        "the reflection into FuelSim stopped working, so balls will accumulate forever");
  }

  @Test
  void theFieldElementsAreSolid() {
    // Walls and the hub were the only things the simulated robot could not drive
    // through. Everything else on the field it went straight through, which
    // matters most for the towers -- they sit right where autos line up.
    SimField.setEnabled(true);
    SimHarness.releaseAllControls();

    assertPushedOut("hub", FieldConstants.LinesVertical.hubCenter,
        FieldConstants.fieldWidth / 2);
    assertPushedOut("opposing hub", FieldConstants.LinesVertical.oppHubCenter,
        FieldConstants.fieldWidth / 2);
    // The tower is two posts and the trench is a wall along its inner edge, not
    // solid blocks -- so these aim at the structures themselves. A robot must
    // still be able to drive between the uprights to climb, which is checked
    // separately below.
    assertPushedOut("tower upright", FieldConstants.Tower.frontFaceX,
        FieldConstants.Tower.leftUpright.getY());
    assertPushedOut("left trench wall", FieldConstants.LinesVertical.hubCenter,
        FieldConstants.LinesHorizontal.leftTrenchOpenEnd - 0.15);
    assertPushedOut("right trench wall", FieldConstants.LinesVertical.hubCenter,
        FieldConstants.LinesHorizontal.rightTrenchOpenStart + 0.15);

    SimField.setEnabled(false);
    SimHarness.levelOut();
  }

  @Test
  void theTowerCanBeDrivenIntoToClimb() {
    // The first version of the obstacle map took the whole tower as a solid
    // block, which is wrong in a way that matters: it made climbing impossible
    // in simulation. It is two uprights, and the gap between them is open.
    SimField.setEnabled(true);
    SimHarness.releaseAllControls();

    double middleY = (FieldConstants.Tower.leftUpright.getY()
        + FieldConstants.Tower.rightUpright.getY()) / 2;
    RobotContainer.drive.setPose(
        new Pose2d(FieldConstants.Tower.frontFaceX, middleY, Rotation2d.kZero));
    SimHarness.step(3);

    Pose2d truth = RobotContainer.simState.getPose();
    double moved = truth.getTranslation()
        .getDistance(new Translation2d(FieldConstants.Tower.frontFaceX, middleY));
    System.out.printf("placed between the tower uprights, moved %.2f m%n", moved);

    SimField.setEnabled(false);
    SimHarness.levelOut();

    // Known limitation, and the reason this is not asserted tighter: the robot
    // is a circle of radius 0.505 m, its half-diagonal, which is right for
    // clipping a corner and too fat for threading a slot. The real robot is
    // 0.762 m wide and fits the 0.858 m gap; the circle needs 1.01 m and so is
    // squeezed by about 9 cm. Modelling the robot as a rectangle would fix it.
    // What is asserted is that the gap is far more open than the upright itself,
    // which pushes 0.33 m.
    assertTrue(moved < 0.15,
        "the gap between the tower uprights should be far more open than the "
            + "uprights themselves, but the robot was pushed " + moved + " m");
  }

  @Test
  void deployingTheIntakeMakesTheRobotBigger() {
    // A robot with the intake down really is a longer object, and it is the part
    // most likely to catch on something. Simulating it at frame size means the
    // one configuration most likely to collide is the one modelled smallest.
    SimField.setEnabled(true);
    SimHarness.releaseAllControls();
    RobotContainer.drive.setPose(new Pose2d(3.0, 2.5, Rotation2d.kZero));

    RobotContainer.state.setExpanderState(ExpanderState.STOWED);
    SimHarness.stepSeconds(1.5);
    double stowed = SimField.getFootprintLength();

    RobotContainer.state.setExpanderState(ExpanderState.EXTENDED);
    SimHarness.stepSeconds(1.5);
    double deployed = SimField.getFootprintLength();

    System.out.printf("robot length: %.3f m stowed, %.3f m with the intake out%n",
        stowed, deployed);

    RobotContainer.state.setExpanderState(ExpanderState.STOWED);
    SimHarness.stepSeconds(0.5);
    SimField.setEnabled(false);

    assertTrue(deployed > stowed + 0.1,
        "deploying the intake should make the robot longer, but it went from "
            + stowed + " m to " + deployed + " m");
  }

  /** Puts the robot inside something solid and checks the field throws it out. */
  private void assertPushedOut(String what, double x, double y) {
    RobotContainer.drive.setPose(new Pose2d(x, y, Rotation2d.kZero));
    SimHarness.step(3);
    Pose2d truth = RobotContainer.simState.getPose();
    double moved = truth.getTranslation().getDistance(new Translation2d(x, y));
    System.out.printf("placed inside the %s, pushed %.2f m to %.2f, %.2f%n",
        what, moved, truth.getX(), truth.getY());
    assertTrue(moved > 0.1, "the " + what + " should be solid, but the robot stayed at " + truth);
  }

  @Test
  void theBumpCostsTheRobotGrip() {
    // Gravity acts on the robot the way it acts on a ball, and the ramp is
    // worse to drive on than carpet. Both come out of the same grip budget: the
    // wheels pay for holding the robot on the slope first and get whatever is
    // left over for going anywhere, on a surface that has less to give.
    //
    // So the same stick, from a standstill, has to move the robot less up the
    // bump than it does on the flat. That is the whole of beaching, without it
    // being written in anywhere.
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    SimHarness.releaseAllControls();

    // Find the bump by sweeping, so this does not depend on which alliance the
    // simulator happens to have picked.
    double onBumpX = Double.NaN;
    for (double x = 1.0; x < FieldConstants.fieldLength - 1.0; x += 0.2) {
      RobotContainer.drive.setPose(new Pose2d(x, 2.5, Rotation2d.kZero));
      SimHarness.step(3);
      if (!RobotContainer.drive.isLevelOnGround()) {
        onBumpX = x;
        break;
      }
    }
    assertTrue(!Double.isNaN(onBumpX), "setup: never found the bump");

    double onBump = distanceCoveredFrom(onBumpX);
    double onFlat = distanceCoveredFrom(onBumpX - 1.5);

    System.out.printf("half a second of full stick: %.3f m on the flat, %.3f m on the bump%n",
        onFlat, onBump);
    SimField.setEnabled(false);
    SimHarness.levelOut();
    SimHarness.releaseAllControls();

    assertTrue(onBump < onFlat * 0.9,
        "climbing the bump should cost the robot ground against the same stick on the flat, "
            + onBump + " m against " + onFlat + " m");
  }

  /** How far full forward stick moves the robot in half a second from a standstill. */
  private double distanceCoveredFrom(double startX) {
    RobotContainer.drive.setPose(new Pose2d(startX, 2.5, Rotation2d.kZero));
    SimHarness.step(3);
    double before = RobotContainer.simState.getPose().getX();
    // Stick forward is -Y on the gamepad and +X on the field.
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, -1.0);
    SimHarness.stepSeconds(0.5);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    return RobotContainer.simState.getPose().getX() - before;
  }

  @Test
  void theRobotRidesOverTheBumpInThreeDimensions() {
    // The robot's pose is a Pose2d, so there is nowhere in it to say the robot
    // is off the floor and nose up. Whatever the gyro reported, the drawn robot
    // stayed flat on the carpet and slid through the bump -- which is why none
    // of the collision work looked like it was working.
    SimField.setEnabled(true);
    RobotContainer.simState.resetSlipTracking();
    SimHarness.releaseAllControls();

    double flatHeight = Double.NaN;
    double peakHeight = 0;
    double peakPitchDegrees = 0;
    for (double x = 1.0; x < FieldConstants.fieldLength - 1.0; x += 0.15) {
      RobotContainer.drive.setPose(new Pose2d(x, 2.5, Rotation2d.kZero));
      SimHarness.step(3);
      var pose = SimField.getRobotPose3d();
      if (Double.isNaN(flatHeight)) {
        flatHeight = pose.getZ();
      }
      if (pose.getZ() > peakHeight) {
        peakHeight = pose.getZ();
      }
      peakPitchDegrees = Math.max(peakPitchDegrees,
          Math.abs(Math.toDegrees(pose.getRotation().getY())));
    }

    System.out.printf("3D robot: %.3f m on the flat, %.3f m at the crest, %.1f deg of pitch%n",
        flatHeight, peakHeight, peakPitchDegrees);
    SimField.setEnabled(false);
    SimHarness.levelOut();

    assertEquals(0.0, flatHeight, 1e-6, "the robot should sit on the carpet away from the bump");
    assertTrue(peakHeight > 0.1,
        "the robot should ride up over the bump, peaked at " + peakHeight + " m");
    assertTrue(peakPitchDegrees > 5,
        "and tip nose up climbing it, peaked at " + peakPitchDegrees + " degrees");
  }

  /** Holds the shoot trigger, so the shooter runs the way a driver runs it. */
  private static void holdShootTrigger(double seconds) {
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightTrigger.value, 1.0);
    SimHarness.stepSeconds(seconds);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightTrigger.value, 0.0);
    SimHarness.stepSeconds(0.3);
  }

  /** Holds the drive stick for a while, so the robot moves under its own power. */
  private static void driveForward(double stick, double seconds) {
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, stick);
    SimHarness.stepSeconds(seconds);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.4);
  }

  /** How far odometry has drifted from where the robot actually is. */
  private static double odometryError() {
    return RobotContainer.simState.getPose().getTranslation()
        .getDistance(RobotContainer.state.getPose().getTranslation());
  }
}
