// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.adaptable.AdaptableManager;

/** Runs Adaptable Standard's direct depot route, in real time. */
public class AdaptableDepotSimTest {
  @BeforeAll
  static void boot() {
    SimHarness.boot();
  }

  @Test
  void drivesStraightToTheDepotAndShoots() {
    SimHarness.setAlliance(Alliance.Blue);
    NetworkTableInstance.getDefault().getTable("SmartDashboard/AdaptableStandard/Route")
        .getEntry("selected").setString("Center to Depot");
    SmartDashboard.updateValues();
    SimHarness.step(2);

    SimHarness.enableAutonomous();
    AdaptableManager.get().schedule();

    double furthestX = 0;
    double arrivedAt = -1;
    Translation2d depot = new Translation2d(0.6, 5.95);

    for (int loop = 0; loop < 8 * 50; loop++) {
      SimHarness.step(1);
      Translation2d pose = RobotContainer.state.getPose().getTranslation();
      furthestX = Math.max(furthestX, pose.getX());
      if (arrivedAt < 0 && pose.getDistance(depot) < 0.2) {
        arrivedAt = loop / 50.0;
      }
    }

    Translation2d end = RobotContainer.state.getPose().getTranslation();
    System.out.println("Depot auto: furthest x = " + furthestX + ", arrived at depot at t = " + arrivedAt);

    assertTrue(furthestX < 3.6, "should never head for the bump, got x = " + furthestX);
    assertTrue(arrivedAt > 0, "should reach the depot");
    assertEquals(0.6, end.getX(), 0.2, "should stay at the depot");
    assertEquals(5.95, end.getY(), 0.2, "should stay at the depot");
    assertTrue(RobotContainer.state.isShooting(), "should be shooting at the depot");

    SimHarness.disable();
  }
}
