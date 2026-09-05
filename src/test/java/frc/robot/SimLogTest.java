// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Proves the full loop: drive the robot in simulation, flush the log, then read
 * it back and assert on logged fields. This is how an agent verifies behaviour
 * that a subsystem does not expose through a public getter.
 */
public class SimLogTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @Test
  void aSimRunProducesAReadableLogWithAdvantageKitFields() {
    SimHarness.enableTeleop();
    RobotContainer.flywheel.runSetpoint(35.0);
    SimHarness.stepSeconds(1.5);
    RobotContainer.flywheel.runSetpoint(0.0);
    SimHarness.disable();

    // Close the log so it can be read back in the same JVM.
    SimHarness.shutdown();

    SimLog log = SimLog.openLatest();
    assertFalse(log.fields().isEmpty(), "log should contain fields");

    // Inputs come from the IO layers, RealOutputs from recordOutput and
    // @AutoLogOutput. Both must be present for a log to be worth reading.
    assertFalse(log.fieldsMatching("/Inputs/").isEmpty(), "expected Inputs fields");
    assertFalse(log.fieldsMatching("/RealOutputs/").isEmpty(), "expected RealOutputs fields");

    // An @AutoLogOutput field, which only appears because SimHarness registers
    // the robot with AutoLogOutputManager the way LoggedRobot normally would.
    var goal = log.maxDouble("/RealOutputs/Flywheel/Flywheel Goal Velocity");
    assertTrue(goal.isPresent(),
        "goal velocity should be logged; Flywheel fields were " + log.fieldsMatching("Flywheel"));
    assertTrue(goal.getAsDouble() > 30.0,
        "logged goal should reach the commanded 35 rps, was " + goal.getAsDouble());

    // And the debounced trigger, proving the sim physics actually spun up.
    assertTrue(log.everTrue("/RealOutputs/Flywheel/At Setpoint"),
        "flywheel should have reached its setpoint at some point during the run");
  }
}
