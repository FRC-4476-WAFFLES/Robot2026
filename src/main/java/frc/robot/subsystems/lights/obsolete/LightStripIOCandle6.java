// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights.obsolete;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import frc.robot.data.Constants;
import frc.robot.subsystems.lights.Lights.LedRange;

public class LightStripIOCandle6 implements LightStripIO {
  private final int LED_COUNT;
  private final CANdle candle = new CANdle(Constants.CANIds.CANdle);
  private final SolidColor solidColorRequest = new SolidColor(0, 0);

  private Map<Integer, RGBWColor> colorMap = new HashMap<>(20);

  private int[] lastSentLEDs;

  public LightStripIOCandle6(int ledCount) {
    LED_COUNT = ledCount;

    lastSentLEDs = new int[LED_COUNT];
    Arrays.fill(lastSentLEDs, 0);

    validateLedRanges();

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    config.LED.BrightnessScalar = 0.75;
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.On;

    candle.getConfigurator().apply(config);

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
  private final void sendBatch(int[] currentLEDs, int start, int end) { // Marked final to make inlining easier for compiler (doesn't really matter)
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
        // int r = (firstColor >> 16) & 0xFF;
        // int g = (firstColor >> 8) & 0xFF;
        // int b = firstColor & 0xFF;
        // candle.setLEDs(r, g, b, 0, chunkStart, chunkSize);
        setLEDs(firstColor, chunkStart, chunkSize);

      } else {
        for (int i = chunkStart; i < chunkEnd; i++) {
          // int r = (color >> 16) & 0xFF;
          // int g = (color >> 8) & 0xFF;
          // int b = color & 0xFF;
          // candle.setLEDs(r, g, b, 0, i, 1);
          setLEDs(currentLEDs[i], i, 1);
        }
      }
    }
  }

  private final RGBWColor getColor(int packed) {
    var val = colorMap.get(packed);
    if (val != null) {
      return val;
    }

    // need to make new color object wth :((
    int r = (packed >> 16) & 0xFF;
    int g = (packed >> 8) & 0xFF;
    int b = packed & 0xFF;

    var color = new RGBWColor(r, g, b, 0);
    colorMap.put(packed, color);
    return color;
  }

  private final void setLEDs(int packedColor, int chunkStart, int chunkSize) {
    candle.setControl(solidColorRequest.withLEDStartIndex(chunkStart).withLEDEndIndex(chunkSize + chunkStart)
        .withColor(getColor(packedColor)));
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
    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
  }
}
