// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.utils.sim.SimBattery;

/**
 * Proves the simulation now reproduces the behaviours the whole power and
 * shooting investigation turned on. Every one of these was untestable before:
 * the simulated flywheel reached any goal at a fixed rate whatever current it
 * was allowed, and there was no battery for a current limit to matter to.
 */
public class SimPhysicsTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @BeforeEach
  void freshPack() {
    SimBattery.reset();
    SimHarness.enableTeleop();
    RobotContainer.flywheel.runSetpoint(0);
    SimHarness.stepSeconds(0.4);
  }

  @AfterEach
  void stopTheWheel() {
    RobotContainer.flywheel.runSetpoint(0);
    SimBattery.reset();
    SimHarness.step(2);
  }

  @Test
  void drawingCurrentSagsTheBus() {
    double idle = SimBattery.getVoltage();
    SimBattery.setLoad("test", 200);
    double loaded = SimBattery.getVoltage();
    assertTrue(loaded < idle - 2.5,
        "200A should cost about 3.1V of bus, went from " + idle + " to " + loaded);
    SimBattery.reset();
  }

  @Test
  void spinningUpLoadsTheBattery() {
    // Measured during spin-up rather than at speed. A wheel sitting at its goal
    // is close to the ceiling, so there is little headroom and it draws almost
    // nothing -- which is the whole reason raising a current limit does nothing
    // at setpoint and everything during a dip.
    RobotContainer.flywheel.runSetpoint(45);
    SimHarness.step(4);
    double spinningUp = SimBattery.getTotalCurrent();
    double busUnderLoad = SimBattery.getVoltage();

    SimHarness.stepSeconds(2.5);
    double atSpeed = SimBattery.getTotalCurrent();

    System.out.printf("flywheel draw: %.0fA spinning up, %.0fA at speed, bus %.2fV%n",
        spinningUp, atSpeed, busUnderLoad);
    assertTrue(spinningUp > 20,
        "spinning up should pull real current, saw " + spinningUp + "A");
    assertTrue(busUnderLoad < 11.79,
        "and the bus should sag below open circuit while it does");
    assertTrue(atSpeed < spinningUp,
        "a wheel at its goal should draw less than one spinning up");
  }

  @Test
  void aTiredPackLowersWhatTheWheelCanReach() {
    // The finding the shooter work rested on: the ceiling falls with the bus, so
    // a long shot that is reachable on a good pack is not on a bad one.
    double healthy = FlywheelIOSim.speedCeiling(12.0);
    double sagged = FlywheelIOSim.speedCeiling(8.0);
    System.out.printf("ceiling: %.1f rps at 12V, %.1f rps at 8V%n", healthy, sagged);
    assertTrue(sagged < healthy - 15,
        "a sagged bus must lower the ceiling materially");
    assertEquals(69.2, FlywheelIOSim.speedCeiling(7.75), 2.0,
        "the model should reproduce the measured 69 rps at 7.75V");
  }

  @Test
  void aBallTakesEnergyOutOfTheWheel() {
    RobotContainer.flywheel.runSetpoint(45);
    SimHarness.stepSeconds(2.0);
    double before = FlywheelIOSim.getActive().getVelocity();
    assertTrue(before > 35, "setup: the wheel should be up to speed, saw " + before);

    SimHarness.takeShot();
    double after = FlywheelIOSim.getActive().getVelocity();
    assertTrue(after < before - 5,
        "a ball should cost the wheel speed, went " + before + " to " + after);
  }

  @Test
  void theWheelRecoversFromABall() {
    RobotContainer.flywheel.runSetpoint(45);
    SimHarness.stepSeconds(2.0);
    SimHarness.takeShot();
    SimHarness.stepSeconds(1.0);
    assertTrue(FlywheelIOSim.getActive().getVelocity() > 40,
        "the wheel should come back after a ball, saw " + FlywheelIOSim.getActive().getVelocity());
  }

  @Test
  void aStarvedFlywheelRecoversMoreSlowly() {
    // The point of the power manager, and the thing the old simulation could not
    // show at all: there, a current limit changed nothing.
    //
    // Driven through the power manager rather than by setting limits directly,
    // because it reapplies its budget every half second and would overwrite
    // anything set by hand -- which is itself the behaviour we want.
    double starved = recoveryTime(true);
    double fed = recoveryTime(false);
    System.out.printf("recovery after a ball: %.2fs starved by turbo, %.2fs on the normal budget%n",
        starved, fed);
    assertTrue(starved > fed,
        "a starved wheel must take longer to recover: " + starved + "s vs " + fed + "s");
  }

  /**
   * Spins up, takes a ball, and returns how long the wheel took to come back.
   *
   * @param starve whether to hold the turbo override, which cuts the flywheel's
   *     supply allowance to give the drivetrain the battery
   */
  private double recoveryTime(boolean starve) {
    var sim = FlywheelIOSim.getActive();
    RobotContainer.powerManager.setTurboOverride(starve);
    RobotContainer.flywheel.runSetpoint(45);
    // Long enough for the power manager to have applied the budget and the wheel
    // to have settled.
    SimHarness.stepSeconds(3.0);

    double atSpeed = sim.getVelocity();
    SimHarness.takeShot(25);
    System.out.printf("  starve=%s: %.1f rps before the ball, %.1f after%n",
        starve, atSpeed, sim.getVelocity());
    // Measured back to whatever speed the robot had settled at, because the
    // shooter state machine commands the flywheel and a setpoint asked for here
    // does not survive the next loop.
    double target = atSpeed - 2.0;
    double elapsed = 0;
    while (elapsed < 4.0 && sim.getVelocity() < target) {
      SimHarness.step(1);
      elapsed += 0.02;
    }

    RobotContainer.powerManager.setTurboOverride(false);
    RobotContainer.flywheel.runSetpoint(0);
    SimHarness.stepSeconds(0.5);
    return elapsed;
  }
}
