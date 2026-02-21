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
  private final LightStrip strip;

  public enum LedRange {
    CANDLE(0, 8),
    LEFT_SIDE_FULL(8, 67),
    MIDDLE_FULL(67, 128),
    RIGHT_SIDE_FULL(128, 186),
    L1(8, 23),
    L2(8, 38),
    L3(8, 52),
    R1(170, 186),
    R2(155, 186),
    R3(141, 186),
    MIDDLE_LEFT(67, 87),
    MIDDLE_MIDDLE(87, 107),
    MIDDLE_RIGHT(107, 128);

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

  public Lights() {
    strip = new LightStrip(186);
  }

  @Override
  public void latePeriodic() {
    EpochTimer.BeginEpoch("LightSubsystem");
    strip.initFrame();

    if (RobotContainer.state.robotEnabled()) {
      handleEnabledState();
    } else {
      handleDisabledState();
    }

    strip.writeFrameToHardware();
    EpochTimer.EndEpoch("LightSubsystem");
  }

  private void handleEnabledState() {

  }

  private void handleDisabledState() {
    updateDiagnosticIndicators();

    strip.setFlowAnimation(LedRange.LEFT_SIDE_FULL, true);
    strip.setFlowAnimation(LedRange.MIDDLE_FULL, true);
    strip.setFlowAnimation(LedRange.RIGHT_SIDE_FULL, true);
  }

  /**
   * Indicators used to perform systems check
   */
  private void updateDiagnosticIndicators() {
    // strip.setRange(0, 1,
    // RobotContainer.intakeSubsystem.isAlgaeLoaded() ?
    // LightColours.DARKGREEN.packed : LightColours.BLACK.packed);

    // strip.setRange(1, 2,
    // RobotContainer.intakeSubsystem.isCoralLoaded() ? LightColours.WHITE.packed :
    // LightColours.BLACK.packed);

    // double pivotPosition =
    // RobotContainer.superstructure.pivot.getPivotPosition();
    // strip.setRange(2, 3, Math.abs(pivotPosition) <= 2.0 ?
    // LightColours.BLUE.packed : LightColours.BLACK.packed);

    // double elevatorPosition =
    // RobotContainer.superstructure.elevator.getElevatorPositionMeters();
    // strip.setRange(3, 4, Math.abs(elevatorPosition) <= 0.02 ?
    // LightColours.CYAN.packed : LightColours.BLACK.packed);

    strip.setRange(4, 5,
        RobotContainer.vision.limelightsSeeTag() ? LightColours.PINK.packed : LightColours.BLACK.packed);

    var alliance = DriverStation.getAlliance();
    LightColours allianceColor = LightColours.BLUE;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      allianceColor = LightColours.RED;
    }
    strip.setRange(6, 8, allianceColor.packed);
  }

  public void celebrationMode() {
    strip.setRainbowAnimation(LedRange.MIDDLE_FULL, true);
  }

  public void clearCelebrationMode() {
    strip.setRainbowAnimation(LedRange.MIDDLE_FULL, false);
  }
}