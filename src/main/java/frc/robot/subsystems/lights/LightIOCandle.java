// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import frc.robot.data.Constants.CANIds;
import frc.robot.subsystems.lights.Lights.LedRange;
import frc.robot.subsystems.lights.Lights.LightColours;

public class LightIOCandle implements LightIO {
  private final CANdle candle;
  private final SolidColor solidColorRequest = new SolidColor(0, 0);
  private final LarsonAnimation disabledRequest = new LarsonAnimation(0, 0);

  private Map<Integer, RGBWColor> colorMap = new HashMap<>(20);

  public LightIOCandle() {
    candle = new CANdle(CANIds.CANdle);
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    config.LED.BrightnessScalar = 0.75;
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.On;

    candle.getConfigurator().apply(config);

    clearHardwareAnimations();
  }

  private RGBWColor getColor(int packed) {
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

  @Override
  public void setLEDs(LightColours color, int chunkStart, int chunkSize) {
    candle.setControl(solidColorRequest.withLEDStartIndex(chunkStart).withLEDEndIndex(chunkSize + chunkStart - 1)
        .withColor(getColor(color.packed)));
  }

  @Override
  public void setLED(LightColours color, int id) {
    candle.setControl(solidColorRequest.withLEDStartIndex(id).withLEDEndIndex(id)
        .withColor(getColor(color.packed)));
  }

  @Override
  public void setLEDs(LightColours color, LedRange range) {
    setLEDs(color, range.getStart(), 1 + range.getEnd() - range.getStart());
  }

  @Override
  public void setDisabledAnimation(LedRange range) {
    candle.setControl(disabledRequest.withLEDEndIndex(range.getStart()).withLEDEndIndex(range.getEnd())
        .withColor(getColor(LightColours.YELLOW.packed)));
  }

  private void clearHardwareAnimations() {
    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
  }
}
