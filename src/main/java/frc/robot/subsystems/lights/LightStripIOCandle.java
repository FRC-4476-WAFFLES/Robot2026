// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.Arrays;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.data.Constants;
import frc.robot.subsystems.lights.Lights.LedRange;

public class LightStripIOCandle implements LightStripIO {
  private final int LED_COUNT;
  private final CANdle candle = new CANdle(Constants.CANIds.CANdle);

  private int[] lastSentLEDs;

  public LightStripIOCandle(int ledCount) {
    LED_COUNT = ledCount;

    lastSentLEDs = new int[LED_COUNT];
    Arrays.fill(lastSentLEDs, 0);

    validateLedRanges();

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 0.75;
    config.vBatOutputMode = VBatOutputMode.On;
    config.v5Enabled = true;
    candle.configAllSettings(config, 1000);

    clearHardwareAnimations();

  }

  /**
  * Send LED data to hardware efficiently (only send changed LEDs)
  */
  @Override
  public void sendColorsToHardware(int[] currentLEDs) {
    int start = -1;

    for (int i = 0; i < LED_COUNT; i++) {
      if (currentLEDs[i] != lastSentLEDs[i]) {
        if (start == -1) {
          start = i;
        }
      } else {
        if (start != -1) {
          sendBatch(currentLEDs, start, i);
          start = -1;
        }
      }
    }

    if (start != -1) {
      sendBatch(currentLEDs, start, LED_COUNT);
    }

    System.arraycopy(currentLEDs, 0, lastSentLEDs, 0, LED_COUNT);
  }

  /**
   * Send a batch of LEDs to hardware efficiently
   */
  private void sendBatch(int[] currentLEDs, int start, int end) {
    int maxChunkSize = 10;

    for (int chunkStart = start; chunkStart < end; chunkStart += maxChunkSize) {
      int chunkEnd = Math.min(chunkStart + maxChunkSize, end);
      int chunkSize = chunkEnd - chunkStart;

      boolean uniformColor = true;
      int firstColor = currentLEDs[chunkStart];
      for (int i = chunkStart + 1; i < chunkEnd; i++) {
        if (currentLEDs[i] != firstColor) {
          uniformColor = false;
          break;
        }
      }

      if (uniformColor && chunkSize > 1) {
        int r = (firstColor >> 16) & 0xFF;
        int g = (firstColor >> 8) & 0xFF;
        int b = firstColor & 0xFF;
        candle.setLEDs(r, g, b, 0, chunkStart, chunkSize);
      } else {
        for (int i = chunkStart; i < chunkEnd; i++) {
          int color = currentLEDs[i];
          int r = (color >> 16) & 0xFF;
          int g = (color >> 8) & 0xFF;
          int b = color & 0xFF;
          candle.setLEDs(r, g, b, 0, i, 1);
        }
      }
    }
  }

  /**
  * Validate that all LED ranges are within bounds
  */
  private void validateLedRanges() {
    for (LedRange range : LedRange.values()) {
      if (range.getStart() < 0 || range.getEnd() > LED_COUNT || range.getStart() >= range.getEnd()) {
        System.err.println("WARNING: Invalid LED range " + range.name() +
            " [" + range.getStart() + "-" + range.getEnd() +
            "] exceeds LED_COUNT=" + LED_COUNT);
      }
    }
  }

  private void clearHardwareAnimations() {
    candle.animate(null);
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }
}
