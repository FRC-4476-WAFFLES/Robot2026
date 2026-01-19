// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.lights.Lights.LedRange;
import frc.robot.subsystems.lights.Lights.LightColours;
import frc.robot.utils.lib.EpochTimer;

/**
 * LED subsystem using array-based approach for reliable, conflict-free operation.
 * 
 * Features:
 * - Blinking for active processes (intake, etc.)
 * - Rainbow and flow animations
 * - Diagnostic indicators
 */
public class LightStrip {
  private final LightStripIO io;

  private final int LED_COUNT;
  private final int FLOW_LENGTH = 15;
  private final double ANIMATION_UPDATE_RATE = 0.05;

  private int[] currentLEDs;

  private final Timer animationTimer = new Timer();
  private int flowPosition = 8;
  private int rainbowOffset = 0;

  private Set<LedRange> rainbowRanges = new HashSet<>();
  private Set<LedRange> flowRanges = new HashSet<>();

  private long lastBlinkTime = 0;
  private boolean blinkState = false;

  public LightStrip(int ledCount) {
    LED_COUNT = ledCount;

    currentLEDs = new int[LED_COUNT];
    Arrays.fill(currentLEDs, 0);

    io = new LightStripIOCandle6(LED_COUNT);

    animationTimer.start();
  }

  /**
  * Clears all previous state & readies for new data to put written to the strip
  */
  public void initFrame() {
    Arrays.fill(currentLEDs, 0);

    if (animationTimer.get() > ANIMATION_UPDATE_RATE) {
      updateAnimations();
      animationTimer.reset();
    }

    updateBlinkState();

    flowRanges.clear();
    rainbowRanges.clear();
  }

  /**
  * Writes the current state of the strip to the hardware
  */
  public void writeFrameToHardware() {
    EpochTimer.BeginEpoch("LightStripIO");
    for (LedRange range : rainbowRanges) {
      applyRainbowRange(range);
    }
    for (LedRange range : flowRanges) {
      applyFlowRange(range);
    }
    io.sendColorsToHardware(currentLEDs);

    EpochTimer.EndEpoch("LightStripIO");
  }

  /**
  * Set a range of LEDs to a specific color
  */
  public void setRange(LedRange range, LightColours color) {
    setRange(range.getStart(), range.getEnd(), color.packed);
  }

  /**
   * Set a range of LEDs with packed RGB value
   */
  public void setRange(int start, int end, int packedColor) {
    if (start < 0 || end < 0 || start > LED_COUNT)
      return;
    int safeStart = Math.max(0, start);
    int safeEnd = Math.min(end, LED_COUNT);
    for (int i = safeStart; i < safeEnd; i++) {
      currentLEDs[i] = packedColor;
    }
  }

  /**
   * Set a range to blink between two colors based on current blink state
   */
  public void setRangeBlinking(LedRange range, LightColours color1, LightColours color2) {
    setRange(range, blinkState ? color1 : color2);
  }

  public void setRainbowAnimation(LedRange range, boolean enabled) {
    if (enabled && range != null) {
      rainbowRanges.add(range);
    } else if (range != null) {
      rainbowRanges.remove(range);
    }
  }

  public void setFlowAnimation(LedRange range, boolean enabled) {
    if (enabled && range != null) {
      flowRanges.add(range);
    } else if (range != null) {
      flowRanges.remove(range);
    }
  }

  /**
   * Set a range to display rainbow spectrum
   */
  private void applyRainbowRange(LedRange range) {
    int start = range.getStart();
    int end = range.getEnd();
    int length = end - start;

    if (length <= 0 || start < 0 || end > LED_COUNT)
      return;

    for (int i = 0; i < length; i++) {
      int ledIndex = start + i;
      if (ledIndex >= LED_COUNT)
        break;
      int hue = ((i + rainbowOffset) * 255 / length) % 255;
      int[] rgb = hsvToRgb(hue, 255, 200);
      currentLEDs[ledIndex] = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
    }
  }

  /**
   * Update animation state (rainbow offset, flow position)
   */
  private void updateAnimations() {
    rainbowOffset = (rainbowOffset + 3) % 255;

    flowPosition = (flowPosition + 2) % 10000;
  }

  /**
   * Update blink state based on time
   */
  private void updateBlinkState() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastBlinkTime > 100) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;
    }
  }

  /**
  * Set a range to display flow animation
  */
  private void applyFlowRange(LedRange range) {
    int start = range.getStart();
    int end = range.getEnd();
    int length = end - start;

    if (length <= 0 || start < 0 || end > LED_COUNT)
      return;

    int cycleLength = length + FLOW_LENGTH; // Add flow length to create smooth wrap
    int relativeFlowPos = flowPosition % cycleLength;

    for (int i = 0; i < FLOW_LENGTH; i++) {
      int ledPos = relativeFlowPos + i - FLOW_LENGTH;

      if (ledPos >= 0 && ledPos < length) {
        int ledIndex = start + ledPos;

        if (ledIndex >= start && ledIndex < end) {
          currentLEDs[ledIndex] = LightColours.FLOW_COLOR.packed;
        }
      }
    }
  }

  private int[] hsvToRgb(int h, int s, int v) {
    double hNorm = (h % 255) * 360.0 / 255.0;
    double hh = hNorm / 60.0;
    int i = (int) hh % 6;
    double ff = hh - (int) hh;
    double p = v * (1.0 - s / 255.0);
    double q = v * (1.0 - (s / 255.0) * ff);
    double t = v * (1.0 - (s / 255.0) * (1.0 - ff));

    switch (i) {
      case 0:
        return new int[] { v, (int) Math.round(t), (int) Math.round(p) };
      case 1:
        return new int[] { (int) Math.round(q), v, (int) Math.round(p) };
      case 2:
        return new int[] { (int) Math.round(p), v, (int) Math.round(t) };
      case 3:
        return new int[] { (int) Math.round(p), (int) Math.round(q), v };
      case 4:
        return new int[] { (int) Math.round(t), (int) Math.round(p), v };
      case 5:
      default:
        return new int[] { v, (int) Math.round(p), (int) Math.round(q) };
    }
  }
}
