// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.intake.Intake.ExpanderState;

/**
 * Exercises the trigger bindings in {@code RobotContainer.configureBindings} by
 * driving the simulated driver station's controllers, which is the only way to
 * cover that code without a human holding a gamepad.
 *
 * <p>
 * The driver is on a single gamepad on port 0, which replaced a pair of flight
 * sticks. Button numbers here are the WPILib Xbox mapping, named rather than
 * written as integers so a remap is a compile error instead of a mystery.
 */
public class ControlsSimTest {
  private static final int A = XboxController.Button.kA.value;
  private static final int BACK = XboxController.Button.kBack.value;
  private static final int START = XboxController.Button.kStart.value;
  private static final int LEFT_TRIGGER_AXIS = XboxController.Axis.kLeftTrigger.value;

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
  void backTogglesManualMode() {
    boolean before = RobotContainer.state.isManualMode();

    SimHarness.tapButton(SimHarness.DRIVER, BACK, 2);
    SimHarness.step(2);
    assertNotEquals(before, RobotContainer.state.isManualMode(), "Back should toggle manual mode");

    SimHarness.tapButton(SimHarness.DRIVER, BACK, 2);
    SimHarness.step(2);
    assertEquals(before, RobotContainer.state.isManualMode(), "pressing again should toggle back");
  }

  @Test
  void aTogglesTheIntake() {
    // Start from a known state rather than trusting whatever ran before.
    RobotContainer.state.setExpanderState(ExpanderState.STOWED);
    SimHarness.step(2);

    SimHarness.tapButton(SimHarness.DRIVER, A, 2);
    SimHarness.step(2);
    assertEquals(ExpanderState.EXTENDED, RobotContainer.state.getExpanderState(),
        "A should extend the intake");

    SimHarness.tapButton(SimHarness.DRIVER, A, 2);
    SimHarness.step(2);
    assertEquals(ExpanderState.STOWED, RobotContainer.state.getExpanderState(),
        "pressing again should stow it");
  }

  @Test
  void bothSticksReachTheDriveInputs() {
    // Translation is the left stick, rotation the right. Getting these crossed
    // is the mistake this catches.
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftX.value, 0.75);
    assertEquals(-0.75, Controls.getDriveXRaw(), 0.02, "left stick X should strafe");

    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.75);
    assertEquals(-0.75, Controls.getDriveYRaw(), 0.02, "left stick Y should drive forward");

    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightX.value, 0.75);
    assertEquals(-0.75, Controls.getDriveRotationRaw(), 0.02, "right stick X should rotate");

    // The left stick must not leak into rotation.
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightX.value, 0.0);
    assertEquals(0.0, Controls.getDriveRotationRaw(), 0.02);
  }

  @Test
  void smallStickMovementIsSwallowedByTheDeadzone() {
    // JOYSTICK_DEADZONE_INNER is 0.025, so this must read as exactly zero. Worth
    // watching now the driver is on a thumbstick, which rests less cleanly at
    // centre than a flight stick did.
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kRightX.value, 0.01);
    assertEquals(0.0, Controls.getDriveRotationRaw(), 1e-9, "inside the inner deadzone");
  }

  @Test
  void theLeftTriggerRunsTheIntakeRollers() {
    SimHarness.setAxis(SimHarness.DRIVER, LEFT_TRIGGER_AXIS, 1.0);
    SimHarness.stepSeconds(0.3);
    assertEquals(ExpanderState.INTAKING, RobotContainer.state.getExpanderState(),
        "holding the left trigger should put the intake into its intaking state");

    SimHarness.setAxis(SimHarness.DRIVER, LEFT_TRIGGER_AXIS, 0.0);
    SimHarness.stepSeconds(0.3);
  }

  @Test
  void startResetsTheGyro() {
    SimHarness.tapButton(SimHarness.DRIVER, START, 2);
    SimHarness.step(4);
    assertEquals(0.0, RobotContainer.state.getPose().getRotation().getDegrees(), 2.0,
        "Start should zero the heading");
  }
}
