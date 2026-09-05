// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.LinkedHashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

/**
 * Guards the CAN device map. The expected values below are the IDs and buses
 * that were in Constants.CANIds before it was replaced by {@link Ports}, so a
 * failure here means a device moved.
 */
public class PortsTest {
  private static final String RIO = "rio";
  private static final String CANIVORE = "CANivore";

  @Test
  void portsMatchTheirHistoricalIdAndBus() {
    Map<Ports, String> expected = new LinkedHashMap<>();
    expected.put(Ports.EXPANDER_MOTOR, "14 " + RIO);
    expected.put(Ports.INTAKE_MOTOR_1, "15 " + RIO);
    expected.put(Ports.INTAKE_MOTOR_0, "16 " + RIO);
    expected.put(Ports.CLIMBER_MOTOR, "17 " + RIO);
    expected.put(Ports.INDEXER_MOTOR_1, "18 " + RIO);
    expected.put(Ports.INDEXER_MOTOR_2, "19 " + RIO);
    expected.put(Ports.FEEDER_MOTOR_0, "20 " + RIO);
    expected.put(Ports.FLYWHEEL_MOTOR_0, "21 " + RIO);
    expected.put(Ports.FLYWHEEL_MOTOR_1, "22 " + RIO);
    expected.put(Ports.HOOD_MOTOR, "23 " + RIO);
    expected.put(Ports.CANDLE, "24 " + RIO);
    expected.put(Ports.FEEDER_MOTOR_1, "28 " + RIO);
    expected.put(Ports.TURRET_MOTOR, "25 " + CANIVORE);
    expected.put(Ports.TURRET_ENCODER_0, "26 " + CANIVORE);
    expected.put(Ports.TURRET_ENCODER_1, "27 " + CANIVORE);

    assertEquals(expected.size(), Ports.values().length, "a port was added or removed without updating this test");

    for (var entry : expected.entrySet()) {
      Ports port = entry.getKey();
      assertEquals(entry.getValue(), port.id + " " + port.bus.getName(), port + " moved");
    }
  }

  @Test
  void noTwoPortsShareAnIdOnTheSameBus() {
    assertNull(Ports.findDuplicate());
  }
}
