// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Proves the assumption the Ports migration relies on: the RIO bus is not CAN FD,
 * so deriving isCANFD from the bus matches the value TalonFXIO's raw-ID
 * constructor used to hardcode.
 *
 * <p>
 * HAL must be initialized before any CTRE object is touched, or Phoenix loads the
 * real native library instead of the _Sim one and the test dies with
 * UnsatisfiedLinkError.
 */
public class CanBusSimTest {
  @BeforeAll
  static void setupHAL() {
    assertTrue(HAL.initialize(500, 0), "HAL initialization failed");
  }

  @AfterAll
  static void teardownHAL() {
    HAL.shutdown();
  }

  /**
   * The reason Ports declares CAN FD rather than querying it. If this ever starts
   * reporting false in simulation, the workaround in Ports.isCANFD() can go away.
   */
  @Test
  void isNetworkFdIsUnreliableInSimulation() {
    assertTrue(
        Ports.Bus.RIO.isNetworkFD(),
        "CANBus.isNetworkFD() no longer lies about the RIO bus in sim - revisit Ports.isCANFD()");
  }

  @Test
  void portsDeclaresTheRealBusTypes() {
    assertFalse(Ports.HOOD_MOTOR.isCANFD(), "RIO bus is CAN 2.0");
    assertTrue(Ports.TURRET_MOTOR.isCANFD(), "CANivore is CAN FD");
  }
}
