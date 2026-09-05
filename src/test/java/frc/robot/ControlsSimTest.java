// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.subsystems.intake.Intake.ExpanderState;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Exercises the trigger bindings in {@code RobotContainer.configureBindings} by
 * driving the simulated driver station's controllers, which is the only way to
 * cover that code without a human holding a joystick.
 */
public class ControlsSimTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @BeforeEach
  void cleanControls() {
    SimHarness.enableTeleop();
    SimHarness.releaseAllControls();
  }

  @Test
  void rightJoystickButtonTwoTogglesManualMode() {
    boolean before = RobotContainer.state.isManualMode();

    SimHarness.tapButton(SimHarness.RIGHT_JOYSTICK, 2, 2);
    SimHarness.step(2);
    assertNotEquals(before, RobotContainer.state.isManualMode(), "button 2 should toggle manual mode");

    SimHarness.tapButton(SimHarness.RIGHT_JOYSTICK, 2, 2);
    SimHarness.step(2);
    assertEquals(before, RobotContainer.state.isManualMode(), "pressing again should toggle back");
  }

  @Test
  void leftJoystickButtonFourTogglesTheIntake() {
    // Start from a known state rather than trusting whatever ran before.
    RobotContainer.state.setExpanderState(ExpanderState.STOWED);
    SimHarness.step(2);

    SimHarness.tapButton(SimHarness.LEFT_JOYSTICK, 4, 2);
    SimHarness.step(2);
    assertEquals(ExpanderState.EXTENDED, RobotContainer.state.getExpanderState(),
        "button 4 should extend the intake");

    SimHarness.tapButton(SimHarness.LEFT_JOYSTICK, 4, 2);
    SimHarness.step(2);
    assertEquals(ExpanderState.STOWED, RobotContainer.state.getExpanderState(),
        "pressing again should stow it");
  }

  @Test
  void joystickAxesReachTheDriveInputs() {
    SimHarness.setAxis(SimHarness.LEFT_JOYSTICK, 0, 0.75);
    // Controls negates the raw axis, and 0.75 is well past the deadzone.
    assertEquals(-0.75, Controls.getDriveXRaw(), 0.02);

    SimHarness.setAxis(SimHarness.LEFT_JOYSTICK, 0, 0.0);
    assertEquals(0.0, Controls.getDriveXRaw(), 0.02);
  }

  @Test
  void smallAxisMovementIsSwallowedByTheDeadzone() {
    // JOYSTICK_DEADZONE_INNER is 0.025, so this must read as exactly zero.
    SimHarness.setAxis(SimHarness.RIGHT_JOYSTICK, 0, 0.01);
    assertEquals(0.0, Controls.getDriveRotationRaw(), 1e-9, "inside the inner deadzone");
  }
}
