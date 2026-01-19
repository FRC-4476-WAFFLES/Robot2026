// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import frc.robot.subsystems.lights.Lights.LedRange;
import frc.robot.subsystems.lights.Lights.LightColours;

public interface LightsIO {
  public default void setRange(LedRange range, LightColours color) {
    setRange(range.getStart(), range.getEnd(), color.packed);
  }

  /**
   * Set a range of LEDs with packed RGB value
   */
  public default void setRange(int start, int end, int packedColor) {}

  /**
  * Set a range to blink between two colors based on current blink state
  */
  public default void setRangeBlinking(LedRange range, LightColours color1, LightColours color2) {}
}
