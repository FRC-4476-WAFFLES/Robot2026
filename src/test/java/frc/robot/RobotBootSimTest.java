// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.Timer;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Proves the whole robot can be booted and driven headlessly, and serves as the
 * worked example for writing simulation tests. See {@link SimHarness}.
 */
public class RobotBootSimTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @Test
  void robotBootsWithEverySubsystemConstructed() {
    assertNotNull(RobotContainer.drive, "drive");
    assertNotNull(RobotContainer.turret, "turret");
    assertNotNull(RobotContainer.intake, "intake");
    assertNotNull(RobotContainer.hood, "hood");
    assertNotNull(RobotContainer.indexer, "indexer");
    assertNotNull(RobotContainer.flywheel, "flywheel");
    assertNotNull(RobotContainer.climber, "climber");
    assertNotNull(RobotContainer.vision, "vision");
    assertNotNull(RobotContainer.state.getPose(), "pose");
  }

  @Test
  void steppingAdvancesTheClock() {
    double before = Timer.getTimestamp();
    SimHarness.step(50); // 1 second
    double elapsed = Timer.getTimestamp() - before;

    assertEquals(1.0, elapsed, 0.30, "50 loops should advance the clock ~1 second");
  }

  @Test
  void enabledStateFollowsTheSimulatedDriverStation() {
    SimHarness.disable();
    assertFalse(RobotContainer.state.robotEnabled(), "should read disabled");

    SimHarness.enableTeleop();
    assertTrue(RobotContainer.state.robotEnabled(), "should read enabled");
    assertFalse(RobotContainer.state.autonomousEnabled(), "teleop is not autonomous");

    SimHarness.enableAutonomous();
    assertTrue(RobotContainer.state.autonomousEnabled(), "should read autonomous");

    SimHarness.disable();
  }

  @Test
  void flywheelReachesItsSetpointWhenEnabled() {
    SimHarness.enableTeleop();

    RobotContainer.flywheel.runSetpoint(40.0);
    SimHarness.stepSeconds(3.0);
    assertTrue(RobotContainer.flywheel.atSetpoint(), "flywheel should reach 40 rps within 3s of sim time");

    RobotContainer.flywheel.runSetpoint(0.0);
    SimHarness.disable();
  }
}
