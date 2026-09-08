// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.sim.SimBattery;
import frc.robot.utils.sim.SimField;
import frc.robot.utils.sim.SimShooter;

/**
 * Checks the simulation against numbers measured on real match logs.
 *
 * <p>
 * A simulation is only worth what it agrees with. These are not invented
 * tolerances — every bound comes from {@code docs/log-deep-dive.md}, measured
 * across 25 FMS matches, and the point is to fail when a change to the sim
 * quietly stops reproducing the robot.
 *
 * <p>
 * The relationships checked here are the ones the offseason work turns on. If
 * the simulated bus does not sag when the drivetrain pulls, then the power
 * manager cannot be evaluated in simulation at all, and the whole reason for
 * building the battery model is gone.
 */
public class SimFidelityTest {
  // Known limitation, left visible rather than hidden: the simulated drivetrain
  // still spikes for a loop or two at the instant full stick is applied, which
  // drives the pack to its 4 V floor. Steady-state loads are sane (7-11 A per
  // module), and the supply limit now clearly bites -- a 10 A limit peaks at
  // ~115 A and holds 10.0 V where 45 A floors the bus. Until the transient is
  // fixed, assert on current rather than voltage, because voltage saturates
  // against the floor and silently compares nothing.

  @BeforeAll
  static void boot() {
    SimHarness.boot();
    SimHarness.enableTeleop();
  }

  @AfterAll
  static void putItBack() {
    SimShooter.setEnabled(false);
    SimField.setEnabled(false);
    SimHarness.releaseAllControls();
    SimHarness.levelOut();
  }

  /**
   * Driving hard must pull the bus down.
   *
   * <p>
   * This is the mechanism behind the season's biggest measured finding: the
   * flywheel is voltage limited, not current limited, and it is the drivetrain
   * that takes the voltage away. On the real robot the bus sits at 10.0 V while
   * the wheel is at setpoint and 7.8 V once it is more than 10 rps down, and
   * the drivetrain is drawing 21 A and 125 A respectively at those moments.
   *
   * <p>
   * The absolute numbers are not asserted — the simulated drivetrain is not
   * fitted that closely — but the direction and rough scale are, because a
   * simulation where driving is free would say the power manager does nothing.
   */
  @Test
  void drivingPullsTheBusDown() {
    SimShooter.setEnabled(false);
    SimField.setEnabled(true);
    RobotContainer.drive.setPose(new Pose2d(3.0, 2.5, Rotation2d.kZero));
    SimHarness.releaseAllControls();
    SimHarness.stepSeconds(1.0);

    double idleVolts = SimBattery.getVoltage();

    // Full stick, and hold it long enough for the current to be real rather
    // than the first loop of a profile.
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, -1.0);
    double lowest = idleVolts;
    double peakAmps = 0;
    for (int i = 0; i < 40; i++) {
      SimHarness.step(1);
      lowest = Math.min(lowest, SimBattery.getVoltage());
      peakAmps = Math.max(peakAmps, SimBattery.getTotalCurrent());
    }
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.5);

    System.out.printf("bus: %.2f V idle, %.2f V driving (sag %.2f V), peak %.0f A%n",
        idleVolts, lowest, idleVolts - lowest, peakAmps);
    SimField.setEnabled(false);

    assertTrue(idleVolts - lowest > 0.25,
        "driving should sag the simulated bus, but it only moved "
            + (idleVolts - lowest) + " V — the power manager cannot be evaluated "
            + "in a simulation where driving is free");
  }

  /**
   * Capping the drivetrain must give the voltage back.
   *
   * <p>
   * The whole power-manager thesis in one assertion: across 159 brownouts the
   * drivetrain's median draw was 176 A of a 188 A total, so cutting its supply
   * limit should be visible at the battery. If this does not hold in
   * simulation, the simulation cannot answer the question the power manager
   * exists to answer.
   */
  @Test
  void cappingTheDrivetrainGivesVoltageBack() {
    SimShooter.setEnabled(false);
    SimField.setEnabled(true);
    SimHarness.releaseAllControls();

    double[] at45 = driveHardWith(45);
    double[] at10 = driveHardWith(10);

    System.out.printf("drivetrain draw: %.0f A at a 45 A/module limit, %.0f A at 10 A%n",
        at45[0], at10[0]);
    SimField.setEnabled(false);
    SimHarness.releaseAllControls();
    RobotContainer.drive.applyCurrentLimits(45);

    // Current rather than voltage, because the simulated pack has a floor and a
    // voltage comparison silently saturates against it — which is exactly how
    // the first version of this test managed to pass nothing useful.
    assertTrue(at10[0] < at45[0] * 0.9,
        "a tighter drive supply limit should draw less current, but 10 A peaked at "
            + at10[0] + " A against " + at45[0] + " A at 45 A");
  }

  /** Peak total current and lowest bus voltage under full stick at a given limit. */
  private double[] driveHardWith(double supplyLimit) {
    RobotContainer.drive.applyCurrentLimits(supplyLimit);
    RobotContainer.drive.setPose(new Pose2d(3.0, 2.5, Rotation2d.kZero));
    SimHarness.releaseAllControls();
    SimHarness.stepSeconds(1.0);
    double idle = SimBattery.getVoltage();

    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, -1.0);
    double lowest = idle;
    double peakAmps = 0;
    int peakAt = -1;
    String peakLoads = "";
    for (int i = 0; i < 40; i++) {
      SimHarness.step(1);
      lowest = Math.min(lowest, SimBattery.getVoltage());
      // The drivetrain specifically, not the whole robot -- the flywheel spins
      // up alongside and would otherwise dominate the number being asserted on.
      double driveAmps = SimBattery.getTotalCurrent("Module");
      if (driveAmps > peakAmps) {
        peakAmps = driveAmps;
        peakAt = i;
        peakLoads = SimBattery.describeLoads();
      }
    }
    System.out.printf("   limit %.0f A/module: peak drivetrain %.0f A on loop %d -> %s%n",
        supplyLimit, peakAmps, peakAt, peakLoads);
    SimHarness.setAxis(SimHarness.DRIVER, XboxController.Axis.kLeftY.value, 0.0);
    SimHarness.stepSeconds(0.5);
    return new double[] { peakAmps, lowest };
  }
}
