// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import frc.robot.subsystems.lights.Lights.LedRange;
import frc.robot.subsystems.lights.Lights.LightColours;

public interface LightIO {
  default void setLEDs(LightColours color, int chunkStart, int chunkSize) {}

  default void setLED(LightColours color, int id) {}

  default void setLEDs(LightColours color, LedRange range) {}

  default void setDisabledAnimation(LedRange range) {}
}
