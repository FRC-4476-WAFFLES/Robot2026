// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class Lights extends VirtualSubsystem {
  private final LightIO io;

  public enum LedRange { // Ranges are inclusive
    CANDLE(0, 7),
    MAIN_BAR(8, 80);

    private final int start;
    private final int end;

    LedRange(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getStart() {
      return start;
    }

    public int getEnd() {
      return end;
    }
  }

  public enum LightColours {
    BLACK(0, 0, 0),
    WHITE(255, 255, 255),
    GRAY(127, 127, 127),
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    YELLOW(255, 190, 0),
    ORANGE(255, 18, 0),
    PURPLE(150, 0, 255),
    CYAN(0, 255, 179),
    PINK(255, 0, 255),
    DARKGREEN(21, 102, 13),
    LIGHTGREEN(130, 247, 119),
    LIGHTRED(255, 105, 105),
    LIGHTBLUE(103, 120, 214),
    NAVY(9, 15, 79),
    DIM_YELLOW(64, 48, 0),
    DIM_PURPLE(38, 0, 64),
    BROWN(96, 32, 8),
    INFRARED(50, 0, 0),
    SUN(255, 60, 0),
    LIME(187, 255, 0),
    ULTRAVIOLET(50, 0, 100),
    MAGENTA(150, 15, 92),
    FLOW_COLOR(255, 190, 0);

    public final int red, green, blue;
    public final int packed;

    LightColours(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.packed = (red << 16) | (green << 8) | blue;
    }
  }

  public Lights(LightIO io) {
    this.io = io;
  }

  @Override
  public void latePeriodic() {
    EpochTimer.BeginEpoch("LightSubsystem");

    if (RobotContainer.state.robotEnabled()) {
      handleEnabledState();
    } else {
      handleDisabledState();
    }

    EpochTimer.EndEpoch("LightSubsystem");
  }

  private void handleEnabledState() {
    updateDiagnosticIndicators();

    if (RobotContainer.state.isManualMode()) {
      io.setLEDs(LightColours.GREEN, LedRange.MAIN_BAR);
    } else {
      if (RobotContainer.state.canFire().getAsBoolean()) {
        io.setLEDs(LightColours.WHITE, LedRange.MAIN_BAR);
      } else {
        io.setLEDs(LightColours.RED, LedRange.MAIN_BAR);
      }
    }
  }

  private void handleDisabledState() {
    updateDiagnosticIndicators();

    io.setDisabledAnimation(LedRange.MAIN_BAR);
  }

  /**
   * Indicators used to perform systems check
   */
  private void updateDiagnosticIndicators() {
    io.setLED(RobotContainer.vision.limelightsSeeTag() ? LightColours.PINK : LightColours.BLACK, 0);

    var alliance = DriverStation.getAlliance();
    LightColours allianceColor = LightColours.BLUE;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      allianceColor = LightColours.RED;
    }
    io.setLEDs(allianceColor, 4, 2);
  }
}