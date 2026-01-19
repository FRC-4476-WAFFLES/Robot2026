// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;
import frc.robot.subsystems.groundsuperstructure.GroundIntakeSuperstructure.GroundIntakeSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.utils.lib.WafflesUtilities;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

/**
 * Simple LED subsystem using array-based approach for reliable, conflict-free operation.
 * 
 * Features:
 * - Blinking for active processes (intake, etc.)
 * - Rainbow and flow animations
 * - Diagnostic indicators
 */
@SuppressWarnings("removal") // TODO: Update this logic
public class Lights extends VirtualSubsystem {
  private static final int LED_COUNT = 186;
  private static final int FLOW_LENGTH = 15;
  private static final double ANIMATION_UPDATE_RATE = 0.05;

  private static final CANdle candle = new CANdle(Constants.CANIds.CANdle);

  private int[] currentLEDs = new int[LED_COUNT];
  private int[] lastSentLEDs = new int[LED_COUNT];

  private static final Timer animationTimer = new Timer();
  private int flowPosition = 8;
  private int rainbowOffset = 0;

  private Set<LedRange> rainbowRanges = new HashSet<>();
  private Set<LedRange> flowRanges = new HashSet<>();

  private long lastBlinkTime = 0;
  private boolean blinkState = false;
  private boolean isCoralIntakeRunning = false;

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
    validateLedRanges();

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 0.75;
    config.vBatOutputMode = VBatOutputMode.On;
    config.v5Enabled = true;
    candle.configAllSettings(config, 1000);

    clearHardwareAnimations();

    animationTimer.start();
    Arrays.fill(currentLEDs, 0);
    Arrays.fill(lastSentLEDs, 0);
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

  @Override
  public void latePeriodic() {
    if (RobotBase.isSimulation())
      return;

    Arrays.fill(currentLEDs, 0);

    if (animationTimer.get() > ANIMATION_UPDATE_RATE) {
      updateAnimations();
      animationTimer.reset();
    }

    updateBlinkState();

    if (DriverStation.isEnabled()) {
      handleEnabledState();
    } else {
      handleDisabledState();
    }

    sendLEDsToHardware();
  }

  /**
   * Set a range of LEDs to a specific color
   */
  private void setRange(LedRange range, LightColours color) {
    setRange(range.getStart(), range.getEnd(), color.packed);
  }

  /**
   * Set a range of LEDs with packed RGB value
   */
  private void setRange(int start, int end, int packedColor) {
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
  private void setRangeBlinking(LedRange range, LightColours color1, LightColours color2) {
    setRange(range, blinkState ? color1 : color2);
  }

  /**
   * Set a range to display rainbow spectrum
   */
  private void setRangeRainbow(LedRange range) {
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
   * Send LED data to hardware efficiently (only send changed LEDs)
   */
  private void sendLEDsToHardware() {
    int start = -1;

    for (int i = 0; i < LED_COUNT; i++) {
      if (currentLEDs[i] != lastSentLEDs[i]) {
        if (start == -1) {
          start = i;
        }
      } else {
        if (start != -1) {
          sendBatch(start, i);
          start = -1;
        }
      }
    }

    if (start != -1) {
      sendBatch(start, LED_COUNT);
    }

    System.arraycopy(currentLEDs, 0, lastSentLEDs, 0, LED_COUNT);
  }

  /**
   * Send a batch of LEDs to hardware efficiently
   */
  private void sendBatch(int start, int end) {
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

  private void handleEnabledState() {
    flowRanges.clear();
    rainbowRanges.clear();

    if (!RobotContainer.isOperatorOverride) {
      handleAutomaticElevatorLights();
    } else {
      handleManualElevatorLights();
      enableManualModeRainbow();
    }

    updatePathingIndicators();
    updateOverrideIndicators();

    for (LedRange range : rainbowRanges) {
      setRangeRainbow(range);
    }
  }

  private void updateOverrideIndicators() {
    if (RobotContainer.isOperatorOverride) {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.GREEN);
    } else if (Controls.doNotScore.getAsBoolean()) {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.RED);
    } else {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.BLACK);
    }
  }

  private void handleDisabledState() {
    clearHardwareAnimations();
    updateDiagnosticIndicators();

    rainbowRanges.clear();

    flowRanges.clear();
    flowRanges.add(LedRange.LEFT_SIDE_FULL);
    flowRanges.add(LedRange.MIDDLE_FULL);
    flowRanges.add(LedRange.RIGHT_SIDE_FULL);

    for (LedRange range : flowRanges) {
      setRangeFlow(range);
    }
  }

  /**
   * Set a range to display flow animation
   */
  private void setRangeFlow(LedRange range) {
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

  /**
   * Indicators used to perform systems check
   */
  private void updateDiagnosticIndicators() {
    setRange(0, 1,
        RobotContainer.intakeSubsystem.isAlgaeLoaded() ? LightColours.DARKGREEN.packed : LightColours.BLACK.packed);

    setRange(1, 2,
        RobotContainer.intakeSubsystem.isCoralLoaded() ? LightColours.WHITE.packed : LightColours.BLACK.packed);

    double pivotPosition = RobotContainer.superstructure.pivot.getPivotPosition();
    setRange(2, 3, Math.abs(pivotPosition) <= 2.0 ? LightColours.BLUE.packed : LightColours.BLACK.packed);

    double elevatorPosition = RobotContainer.superstructure.elevator.getElevatorPositionMeters();
    setRange(3, 4, Math.abs(elevatorPosition) <= 0.02 ? LightColours.CYAN.packed : LightColours.BLACK.packed);

    setRange(4, 5,
        RobotContainer.vision.limelightsSeeTag() ? LightColours.PINK.packed : LightColours.BLACK.packed);

    var alliance = DriverStation.getAlliance();
    LightColours allianceColor = LightColours.BLUE;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      allianceColor = LightColours.RED;
    }
    setRange(6, 8, allianceColor.packed);
  }

  /**
   * Updates elevator side lights based on targeted elevator height in automatic mode
   */
  private void handleAutomaticElevatorLights() {
    if (RobotContainer.isHeadingLockedToL1.getAsBoolean()) {
      setRangeBlinking(LedRange.LEFT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK);
      setRangeBlinking(LedRange.RIGHT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK);
      return;
    }
    if (RobotContainer.groundSuperstructure.isL1Ready()) {
      setRange(LedRange.LEFT_SIDE_FULL, LightColours.GREEN);
      setRange(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN);
      return;
    }
    if (RobotContainer.groundSuperstructure.getState() == GroundIntakeSuperstructureState.INTAKE_L1_STATE) {
      setRangeBlinking(LedRange.LEFT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK);
      setRangeBlinking(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK);
      return;
    }

    SuperstructureState scoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    boolean isRightSide = RobotContainer.dynamicPathingSubsystem.getCoralScoringSide();

    switch (scoringLevel) {
      case L1:
        setElevatorSideLights(LedRange.L1, LedRange.R1, isRightSide);
        break;
      case L2:
        setElevatorSideLights(LedRange.L2, LedRange.R2, isRightSide);
        break;
      case L3:
        setElevatorSideLights(LedRange.L3, LedRange.R3, isRightSide);
        break;
      case L4:
        setElevatorSideLights(LedRange.LEFT_SIDE_FULL, LedRange.RIGHT_SIDE_FULL, isRightSide);
        break;
      default:
        break;
    }
  }

  /**
   * Set elevator side lights with appropriate colors based on scoring side
   * Accounts for left/right inversion when robot is on opposite side of reef
   */
  private void setElevatorSideLights(LedRange leftRange, LedRange rightRange, boolean isRightSide) {
    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    Rotation2d closestFaceAngle = RobotContainer.dynamicPathingSubsystem.calculateClosestFaceAngle(robotPose);
    closestFaceAngle = WafflesUtilities.FlipAngleIfRedAlliance(closestFaceAngle);

    double angleDifference = closestFaceAngle.plus(Rotation2d.k180deg).minus(Rotation2d.kZero).getDegrees();
    boolean shouldInvertSides = Math.abs(angleDifference) > 90;

    boolean actualRightSide = shouldInvertSides ? !isRightSide : isRightSide;

    setRange(LedRange.LEFT_SIDE_FULL, LightColours.BLACK);
    setRange(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK);

    if (actualRightSide) {
      setRange(leftRange, LightColours.YELLOW);
      setRange(rightRange, LightColours.BLACK);
      rainbowRanges.add(rightRange);
    } else {
      setRange(leftRange, LightColours.BLACK);
      rainbowRanges.add(leftRange);
      setRange(rightRange, LightColours.YELLOW);
    }
  }

  private void handleManualElevatorLights() {
    SuperstructureState elevatorLevel = RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();
    setElevatorLevelPattern(elevatorLevel, hasCoralLoaded);
  }

  private void setElevatorLevelPattern(SuperstructureState level, boolean isCoralLoaded) {
    LightColours color = isCoralLoaded ? LightColours.WHITE : LightColours.BLACK;

    switch (level) {
      case L1:
      case PROCESSOR:
        setRange(LedRange.L1, color);
        setRange(LedRange.R1, color);
        break;
      case L2:
      case ALGAE_L1:
        setRange(LedRange.L2, color);
        setRange(LedRange.R2, color);
        break;
      case L3:
      case ALGAE_L2:
        setRange(LedRange.L3, color);
        setRange(LedRange.R3, color);
        break;
      case L4:
      case NET:
        setRange(LedRange.LEFT_SIDE_FULL, color);
        setRange(LedRange.RIGHT_SIDE_FULL, color);
        break;
      default:
        setRange(LedRange.LEFT_SIDE_FULL, LightColours.BLACK);
        setRange(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK);
        break;
    }
  }

  private void updatePathingIndicators() {
    var pathingSituation = RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation();

    LightColours color = LightColours.BLACK;
    boolean shouldBlink = false;

    if (isCoralIntakeRunning) {
      color = LightColours.WHITE;
      shouldBlink = true;
    } else if (pathingSituation == DynamicPathingSituation.REEF_CORAL) {
      color = LightColours.WHITE;
    } else if (pathingSituation == DynamicPathingSituation.REEF_ALGAE) {
      color = LightColours.DARKGREEN;
    } else if (pathingSituation == DynamicPathingSituation.PROCESSOR) {
      color = LightColours.BLUE;
    } else if (pathingSituation == DynamicPathingSituation.NET) {
      color = LightColours.PINK;
    } else if (pathingSituation == DynamicPathingSituation.HUNT_CORAL) {
      color = LightColours.ORANGE;
    }

    if (shouldBlink) {
      setRangeBlinking(LedRange.MIDDLE_LEFT, color, LightColours.BLACK);
      setRangeBlinking(LedRange.MIDDLE_RIGHT, color, LightColours.BLACK);
    } else {
      setRange(LedRange.MIDDLE_LEFT, color);
      setRange(LedRange.MIDDLE_RIGHT, color);
    }
  }

  public void setCoralIntakeRunning(boolean running) {
    isCoralIntakeRunning = running;
  }

  public void celebrationMode() {
    rainbowRanges.add(LedRange.MIDDLE_FULL);
  }

  public void clearCelebrationMode() {
    rainbowRanges.remove(LedRange.MIDDLE_FULL);
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

  public void clearAllLEDs() {
    Arrays.fill(currentLEDs, 0);
    Arrays.fill(lastSentLEDs, 0);
    candle.setLEDs(0, 0, 0, 0, 0, LED_COUNT);
    rainbowRanges.clear();
    flowRanges.clear();
  }

  public void clearHardwareAnimations() {
    candle.animate(null);
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }

  private void enableManualModeRainbow() {
    SuperstructureState elevatorLevel = RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();

    if (!hasCoralLoaded) {
      switch (elevatorLevel) {
        case L1:
        case PROCESSOR:
          rainbowRanges.add(LedRange.L1);
          rainbowRanges.add(LedRange.R1);
          break;
        case L2:
        case ALGAE_L1:
          rainbowRanges.add(LedRange.L2);
          rainbowRanges.add(LedRange.R2);
          break;
        case L3:
        case ALGAE_L2:
          rainbowRanges.add(LedRange.L3);
          rainbowRanges.add(LedRange.R3);
          break;
        case L4:
        case NET:
          rainbowRanges.add(LedRange.LEFT_SIDE_FULL);
          rainbowRanges.add(LedRange.RIGHT_SIDE_FULL);
          break;
        default:
          break;
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