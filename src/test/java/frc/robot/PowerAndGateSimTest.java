// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.subsystems.power.PowerManagerState;

/**
 * Drives the power manager and the shot gate on a booted robot, because both are
 * built out of things a unit test cannot reach: a background thread writing
 * motor configurations over CAN, and a debounced trigger that only means anything
 * once the scheduler has been running.
 *
 * <p>
 * The simulated IO layers extend the real TalonFX ones, so a configuration
 * applied here goes through the same {@code PhoenixHelpers.tryConfig} path it
 * would on the robot and reports the same success or failure.
 */
public class PowerAndGateSimTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.enableTeleop();
    // The first budget is written from a background thread, so give it loops to
    // land rather than asserting on the same tick it was requested.
    SimHarness.stepSeconds(1.0);
  }

  @Test
  void thePowerManagerSettlesOnABudget() {
    assertNotNull(RobotContainer.powerManager, "power manager was never constructed");
    assertNotNull(RobotContainer.powerManager.getAppliedState(),
        "no budget ever finished applying, so every CAN write is failing");
  }

  @Test
  void anIdleRobotSitsAtTheDefaultBudget() {
    RobotContainer.powerManager.setTurboOverride(false);
    SimHarness.stepSeconds(1.5);
    assertEquals(PowerManagerState.DEFAULT, RobotContainer.powerManager.getAppliedState(),
        "a robot that is not shooting should be at the limits its subsystems configure themselves");
  }

  @Test
  void theTurboOverrideReachesTheMotors() {
    // Exercises the whole path: a state change on the main loop, handed to the
    // executor, written to every managed motor, and only then reported applied.
    RobotContainer.powerManager.setTurboOverride(true);
    SimHarness.stepSeconds(1.5);
    assertEquals(PowerManagerState.TURBO, RobotContainer.powerManager.getAppliedState(),
        "turbo was requested but never applied");

    RobotContainer.powerManager.setTurboOverride(false);
    SimHarness.stepSeconds(1.5);
    assertEquals(PowerManagerState.DEFAULT, RobotContainer.powerManager.getAppliedState(),
        "releasing turbo must hand the budget back");
  }

  @Test
  void theGateToleranceIsSaneWithNoShotPlanned() {
    // With no target the distance is zero, and a tolerance derived from it would
    // be a divide by zero. It must fall back to the loosest bound instead.
    double tolerance = RobotContainer.flywheel.velocityTolerance();
    assertTrue(tolerance >= FlywheelConstants.MIN_VELOCITY_TOLERANCE
        && tolerance <= FlywheelConstants.MAX_VELOCITY_TOLERANCE,
        "tolerance " + tolerance + " is outside its own bounds");
    assertTrue(!Double.isNaN(tolerance), "tolerance is NaN, so the gate can never open");
  }

  @Test
  void theGateOpensOnceTheWheelSettles() {
    RobotContainer.flywheel.runSetpoint(40.0);
    SimHarness.stepSeconds(3.0);
    assertTrue(RobotContainer.flywheel.atSetpoint(),
        "the wheel reached 40 rps but the gate never opened");
    RobotContainer.flywheel.runSetpoint(0.0);
    SimHarness.stepSeconds(0.5);
  }

  @Test
  void theGateDoesNotSlamShutOnABriefDisturbance() {
    // The whole point of the falling debounce. A ball drags the wheel down a
    // median of 7.8 rps on its way out, after it has already gone, and a gate
    // that closes on that shuts behind every shot.
    RobotContainer.flywheel.runSetpoint(40.0);
    SimHarness.stepSeconds(3.0);
    assertTrue(RobotContainer.flywheel.atSetpoint(), "gate never opened, so this proves nothing");

    // Move the goal far enough that the wheel is instantly outside tolerance.
    RobotContainer.flywheel.runSetpoint(60.0);
    SimHarness.step(2);
    assertTrue(RobotContainer.flywheel.atSetpoint(),
        "the gate closed immediately -- the falling debounce is not doing its job");

    // The other half -- that it does eventually close -- cannot be shown here.
    // FlywheelIOSim is a second-order response driven straight from the setpoint,
    // with no torque, no current limit and no load, so the simulated wheel
    // chases any goal faster than the debounce can expire. Proving a gate closes
    // on a wheel that cannot keep up needs either a physical flywheel model or a
    // real robot.

    RobotContainer.flywheel.runSetpoint(0.0);
    SimHarness.stepSeconds(0.5);
  }

  @Test
  void theDriverIsNotToldToHoldFireWhenNotAskingToShoot() {
    // The rumble means "you asked and I refused". It must not fire simply
    // because the flywheel is idle, or it would buzz for the whole match.
    SimHarness.releaseAllControls();
    SimHarness.stepSeconds(0.6);
    assertFalse(RobotContainer.state.holdingFire().getAsBoolean(),
        "holdingFire must be false when the driver is not asking to shoot");
  }

  @Test
  void theRobotKeepsRunningForSeveralSecondsWithEverythingWired() {
    // The power manager writes configurations from a background thread while the
    // main loop runs. If that threw, or deadlocked against the odometry lock,
    // this is where it would show.
    RobotContainer.powerManager.setTurboOverride(true);
    SimHarness.stepSeconds(1.0);
    RobotContainer.powerManager.setTurboOverride(false);
    SimHarness.stepSeconds(1.0);
    assertTrue(RobotContainer.state.robotEnabled(), "the robot stopped being enabled partway");
    assertNotNull(RobotContainer.powerManager.getAppliedState(), "budget application stopped");
  }
}
