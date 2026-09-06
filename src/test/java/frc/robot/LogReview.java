// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * Streams a match {@code .wpilog} and answers questions about it, for reviewing
 * real robot behaviour rather than simulated behaviour.
 *
 * <pre>
 *   ./gradlew logReview --args="fields &lt;log&gt;"   list every field name
 *   ./gradlew logReview --args="align  &lt;log|dir&gt;" report how alignments ended
 * </pre>
 *
 * <p>
 * Streams record by record and keeps only the fields it needs, because match
 * logs run to 100MB+ and loading one whole would not fit comfortably in memory.
 */
public final class LogReview {
  private LogReview() {}

  private static final String ACTIVE = "/RealOutputs/RobotState/Autopilot/Active";
  private static final String FIELD_VELOCITY = "/RealOutputs/RobotState/FieldVelocity";
  private static final String FIELD_POSE = "/RealOutputs/RobotState/FieldPose";
  private static final String TARGET = "/RealOutputs/RobotState/Autopilot/Target";
  private static final String SIM_TRUTH = "/RealOutputs/Vision/UnderlyingFieldPose";
  private static final String VALIDATED = "/RealOutputs/Vision/Validated Pose";
  private static final String BATTERY_VOLTAGE = "/SystemStats/BatteryVoltage";
  private static final String FLYWHEEL_GOAL = "/RealOutputs/Flywheel/Flywheel Goal Velocity";
  private static final String FLYWHEEL_AT_SETPOINT = "/RealOutputs/Flywheel/At Setpoint";
  private static final String FLYWHEEL_MOTOR = "/Inputs/Flywheel/FlywheelMotorData0";
  private static final String FIRE_SHOT = "/RealOutputs/Commands/Fire shot";
  private static final String SHOOTER_STATE = "/RealOutputs/RobotState/Shooter State";
  private static final String DISTANCE_TO_TARGET = "/RealOutputs/Turret/Distance To Target";
  private static final String BROWNED_OUT = "/SystemStats/BrownedOut";
  private static final String TOTAL_CURRENT = "/PowerDistribution/TotalCurrent";
  private static final String MATCH_TIME = "/DriverStation/MatchTime";
  private static final String ENABLED = "/DriverStation/Enabled";

  public static void main(String[] args) throws IOException {
    if (args.length < 2) {
      System.out
          .println(
              "usage: fields <log> | align <log|dir> | vision <log|dir> | power <log|dir> | channels <log|dir> | motor <name> <log|dir>");
      return;
    }
    String mode = args[0];

    List<File> logs = new ArrayList<>();
    for (int i = mode.equals("motor") || mode.equals("bog") ? 2 : 1; i < args.length; i++) {
      File target = new File(args[i]);
      if (target.isDirectory()) {
        File[] found = target.listFiles(f -> f.getName().endsWith(".wpilog") && f.length() > 1024);
        if (found != null) {
          Arrays.sort(found);
          logs.addAll(Arrays.asList(found));
        }
      } else {
        logs.add(target);
      }
    }

    switch (mode) {
      case "fields" -> listFields(logs.get(0));
      case "power" -> {
        System.out.printf("%-42s %s%n", "log",
            "  volts by match time remaining (>100 / 100-75 / 75-50 / 50-25 / <25)   min   brownouts  peakA");
        for (File log : logs) {
          reviewPower(log);
        }
      }
      case "channels" -> reviewChannels(logs);
      case "motor" -> reviewMotor(args[1], logs);
      case "bog" -> reviewBog(args[1], logs);
      case "shooter" -> reviewShooter(logs);
      case "shots" -> reviewShots(logs);
      case "vision" -> {
        for (File log : logs) {
          reviewVision(log);
        }
      }
      case "align" -> {
        int totalEnds = 0;
        int movingEnds = 0;
        for (File log : logs) {
          int[] result = reviewAlignments(log);
          totalEnds += result[0];
          movingEnds += result[1];
        }
        System.out.println();
        System.out.printf("TOTAL: %d alignment ends, %d ended while moving faster than 0.15 m/s%n",
            totalEnds, movingEnds);
      }
      default -> System.out.println("unknown mode: " + mode);
    }
  }

  private static final String CHANNEL_CURRENT = "/PowerDistribution/ChannelCurrent";
  /** Total current at or above which a sample counts as a spike, in amps. */
  private static final double SPIKE_AMPS = 250;
  /** The supply current limit ModuleIOTalonFX configures on each drive motor. */
  private static final double DRIVE_SUPPLY_LIMIT = 45;
  /** Byte offset of supplyCurrent within the packed TalonFXIOData struct. */
  private static final int SUPPLY_CURRENT_OFFSET = 4 * Double.BYTES;
  /** Byte offsets of the other TalonFXIOData fields this file reads. */
  private static final int VELOCITY_OFFSET = 1 * Double.BYTES;
  private static final int STATOR_CURRENT_OFFSET = 5 * Double.BYTES;
  private static final int DUTY_CYCLE_OFFSET = 6 * Double.BYTES;
  private static final int TEMPERATURE_OFFSET = 7 * Double.BYTES;
  private static final int TORQUE_CURRENT_OFFSET = 3 * Double.BYTES;
  private static final int MOTOR_VOLTAGE_OFFSET = 2 * Double.BYTES;

  /** Running peak/mean for one current source. */
  private static final class Draw {
    double peak;
    double sum;
    long count;
    double spikeSum;
    long spikeCount;
    long overLimit;

    void add(double amps, boolean spike) {
      peak = Math.max(peak, amps);
      if (amps > DRIVE_SUPPLY_LIMIT) {
        overLimit++;
      }
      sum += amps;
      count++;
      if (spike) {
        spikeSum += amps;
        spikeCount++;
      }
    }

    double mean() {
      return count == 0 ? 0 : sum / count;
    }

    double spikeMean() {
      return spikeCount == 0 ? 0 : spikeSum / spikeCount;
    }
  }

  /**
   * Attributes total current draw to its sources, across every log given.
   *
   * <p>
   * Three kinds of source are read: the PDH's per-channel array (the ground
   * truth for what the battery is supplying), every {@code TalonFXIOData}
   * struct's <b>supply</b> current (labelled, so a channel can be identified by
   * matching numbers), and the drive modules' {@code CurrentAmps} — which are
   * <b>stator</b>, not supply, so they read high at low duty cycle and are not
   * battery draw. Compare them against the PDH channels rather than summing them
   * in.
   *
   * <p>
   * The "spike" column is the mean while total draw is at or above
   * {@value #SPIKE_AMPS} A. That column, not the overall mean, is what browns the
   * robot out.
   */
  private static void reviewChannels(List<File> logs) throws IOException {
    Map<String, Draw> draws = new HashMap<>();
    Map<String, Correlation> matches = new HashMap<>();
    Map<String, Double> worstSnapshot = new HashMap<>();
    double[] worstTotal = { 0 };
    long enabledSamples = 0;
    long spikeSamples = 0;

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> talons = new HashMap<>();
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      double battery = 12.0;
      int batteryEntry = -1;
      int channelEntry = -1;
      int currentEntry = -1;
      int enabledEntry = -1;
      boolean enabled = false;
      double total = 0;
      Map<String, Double> latestSupply = new HashMap<>();

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(CHANNEL_CURRENT)) {
              channelEntry = start.entry;
            } else if (start.name.equals(TOTAL_CURRENT)) {
              currentEntry = start.entry;
            } else if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.type.equals("struct:TalonFXIOData")) {
              talons.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("CurrentAmps")) {
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("AppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
            } else if (start.name.equals(BATTERY_VOLTAGE)) {
              batteryEntry = start.entry;
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (entry == currentEntry) {
            total = record.getDouble();
          } else if (!enabled) {
            continue;
          } else if (entry == channelEntry) {
            double[] channels = record.getDoubleArray();
            boolean spike = total >= SPIKE_AMPS;
            enabledSamples++;
            double channelSum = 0;
            for (double amps : channels) {
              channelSum += amps;
            }
            draws.computeIfAbsent("SUM of channels", k -> new Draw()).add(channelSum, spike);
            draws.computeIfAbsent("TotalCurrent", k -> new Draw()).add(total, spike);
            if (total > worstTotal[0]) {
              worstTotal[0] = total;
              worstSnapshot.clear();
              worstSnapshot.putAll(latestSupply);
              for (int i = 0; i < channels.length; i++) {
                worstSnapshot.put(String.format("PDH ch%02d", i), channels[i]);
              }
              worstSnapshot.put("__log " + log.getName(), total);
            }
            if (spike) {
              spikeSamples++;
            }
            for (int i = 0; i < channels.length; i++) {
              String channel = String.format("PDH ch%02d", i);
              draws.computeIfAbsent(channel, k -> new Draw()).add(channels[i], spike);
              for (var supply : latestSupply.entrySet()) {
                matches.computeIfAbsent(channel + "@@" + supply.getKey(), k -> new Correlation())
                    .add(channels[i], supply.getValue());
              }
            }
          } else if (talons.containsKey(entry)) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double supply = Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET));
            draws.computeIfAbsent(talons.get(entry), k -> new Draw()).add(supply, total >= SPIKE_AMPS);
            latestSupply.put(talons.get(entry), supply);
          } else if (statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            draws.computeIfAbsent(base + " stator", k -> new Draw())
                .add(Math.abs(record.getDouble()), total >= SPIKE_AMPS);
            recordSupplyEstimate(base, stator, volts, battery, total, draws, latestSupply);
          } else if (appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
            recordSupplyEstimate(base, stator, volts, battery, total, draws, latestSupply);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("%d enabled PDH samples, %d of them at or above %.0fA total%n%n",
        enabledSamples, spikeSamples, SPIKE_AMPS);
    System.out.printf("%-28s %8s %8s %8s %8s %8s%n",
        "source", "mean", "spike", "peak", "samples", ">45A");
    draws.entrySet().stream()
        .filter(e -> e.getValue().peak >= 1.0)
        .sorted((a, b) -> Double.compare(b.getValue().spikeMean(), a.getValue().spikeMean()))
        .forEach(e -> System.out.printf("%-28s %7.1fA %7.1fA %7.1fA %8d %7.2f%%%n",
            e.getKey(), e.getValue().mean(), e.getValue().spikeMean(), e.getValue().peak,
            e.getValue().count, 100.0 * e.getValue().overLimit / Math.max(1, e.getValue().count)));

    // SUM of channels should track TotalCurrent. Where it does not, the PDH
    // reported something impossible and the total is not to be trusted.
    System.out.printf("%nEverything drawing at the single highest-total sample (%.0fA):%n", worstTotal[0]);
    worstSnapshot.entrySet().stream()
        .sorted((a, b) -> Double.compare(b.getValue(), a.getValue()))
        .filter(e -> e.getValue() >= 1.0)
        .forEach(e -> System.out.printf("  %-32s %6.1fA%n", e.getKey(), e.getValue()));

    // Nothing in the log says which mechanism is wired to which PDH channel, so
    // identify each channel by which logged supply current its trace tracks. A
    // channel whose best r is low is a motor that logs no supply current at all.
    System.out.printf("%n%-12s %-32s %s%n", "channel", "best match", "r");
    Map<String, String> best = new HashMap<>();
    Map<String, Double> bestR = new HashMap<>();
    for (var e : matches.entrySet()) {
      String[] parts = e.getKey().split("@@");
      double r = e.getValue().r();
      if (r > bestR.getOrDefault(parts[0], Double.NEGATIVE_INFINITY)) {
        bestR.put(parts[0], r);
        best.put(parts[0], parts[1]);
      }
    }
    bestR.entrySet().stream()
        .sorted(Map.Entry.comparingByKey())
        .forEach(e -> System.out.printf("%-12s %-32s %.2f%n",
            e.getKey(), best.get(e.getKey()), e.getValue()));
  }

  /** Pearson correlation between two paired series, accumulated as they stream. */
  private static final class Correlation {
    private double sumX;
    private double sumY;
    private double sumXx;
    private double sumYy;
    private double sumXy;
    private long n;

    void add(double x, double y) {
      sumX += x;
      sumY += y;
      sumXx += x * x;
      sumYy += y * y;
      sumXy += x * y;
      n++;
    }

    double r() {
      double covariance = n * sumXy - sumX * sumY;
      double spread = Math.sqrt(n * sumXx - sumX * sumX) * Math.sqrt(n * sumYy - sumY * sumY);
      return spread == 0 ? 0 : covariance / spread;
    }
  }

  /**
   * Estimates a motor's supply current from its stator current and applied
   * voltage, for the drive modules, which log stator current only. A motor is a
   * power converter: it draws {@code stator * appliedVolts / busVoltage} from the
   * battery, so a motor stalled at low duty cycle pulls far less from the battery
   * than its stator current suggests.
   */
  private static void recordSupplyEstimate(String base, Map<String, Double> stator,
      Map<String, Double> volts, double battery, double total,
      Map<String, Draw> draws, Map<String, Double> latestSupply) {
    Double amps = stator.get(base);
    Double applied = volts.get(base);
    if (amps == null || applied == null || battery < 4.0) {
      return;
    }
    double supply = amps * applied / battery;
    draws.computeIfAbsent(base + " supply (est)", k -> new Draw()).add(supply, total >= SPIKE_AMPS);
    latestSupply.put(base + " supply (est)", supply);
  }

  /** Turns "/Inputs/Flywheel/FlywheelMotorData0" into "Flywheel/FlywheelMotorData0". */
  private static String shortName(String name) {
    return name.replace("/Inputs/", "").replace("/Drive/", "").replace("Module", "M");
  }

  /**
   * Buckets one motor's samples by commanded duty cycle and reports what it drew
   * in each bucket. This is how you tell whether a supply current limit would
   * bite: supply current is roughly stator current times duty cycle, so a motor
   * run open-loop at full duty draws supply ~= stator and a supply limit becomes
   * the binding constraint on its torque. A motor held at low duty draws far less
   * supply than stator, and a supply limit barely touches it.
   *
   * @param filter matched against the logged field name, e.g. "Intake"
   */
  private static void reviewMotor(String filter, List<File> logs) throws IOException {
    // Buckets of 0.1 commanded duty cycle, 0.0-0.1 through 0.9-1.0.
    Map<String, Draw[]> supply = new HashMap<>();
    Map<String, Draw[]> stator = new HashMap<>();
    Map<String, Draw[]> speed = new HashMap<>();
    Map<String, Draw[]> temperature = new HashMap<>();
    Map<String, Draw[]> torque = new HashMap<>();
    Map<String, Draw[]> motorVolts = new HashMap<>();

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> talons = new HashMap<>();
      int enabledEntry = -1;
      boolean enabled = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.type.equals("struct:TalonFXIOData") && start.name.contains(filter)) {
              talons.put(start.entry, shortName(start.name));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          if (record.getEntry() == enabledEntry) {
            enabled = record.getBoolean();
          } else if (enabled && talons.containsKey(record.getEntry())) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double duty = Math.abs(buf.getDouble(DUTY_CYCLE_OFFSET));
            int bucket = Math.min(9, (int) (duty * 10));
            String motor = talons.get(record.getEntry());
            supply.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET)), false);
            stator.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(Math.abs(buf.getDouble(STATOR_CURRENT_OFFSET)), false);
            speed.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(buf.getDouble(VELOCITY_OFFSET), false);
            temperature.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(buf.getDouble(TEMPERATURE_OFFSET), false);
            torque.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(buf.getDouble(TORQUE_CURRENT_OFFSET), false);
            motorVolts.computeIfAbsent(motor, k -> newBuckets())[bucket]
                .add(buf.getDouble(MOTOR_VOLTAGE_OFFSET), false);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    if (supply.isEmpty()) {
      System.out.println("no TalonFXIOData field matched " + filter);
      return;
    }

    // Summary first: "active" means commanded above 10% duty, so a motor that
    // spends the match idle is not averaged down to nothing.
    System.out.printf("%-30s %10s %10s %10s %10s %10s %8s %8s%n",
        "motor", "active", "supply", "stator", "torque", "volts", "temp", "temp pk");
    for (String motor : new TreeSet<>(supply.keySet())) {
      Draw[] amps = supply.get(motor);
      Draw[] stat = stator.get(motor);
      double activeSum = 0;
      double statorSum = 0;
      long active = 0;
      long total = 0;
      for (int i = 0; i < 10; i++) {
        total += amps[i].count;
        if (i > 0) {
          activeSum += amps[i].sum;
          statorSum += stat[i].sum;
          active += amps[i].count;
        }
      }
      Draw[] temp = temperature.get(motor);
      double tempSum = 0;
      double tempPeak = 0;
      long tempCount = 0;
      for (int i = 0; i < 10; i++) {
        tempSum += temp[i].sum;
        tempCount += temp[i].count;
        tempPeak = Math.max(tempPeak, temp[i].peak);
      }
      Draw[] torques = torque.get(motor);
      double torqueSum = 0;
      long torqueCount = 0;
      for (int i = 1; i < 10; i++) {
        torqueSum += torques[i].sum;
        torqueCount += torques[i].count;
      }
      Draw[] volts = motorVolts.get(motor);
      double voltSum = 0;
      long voltCount = 0;
      for (int i = 1; i < 10; i++) {
        voltSum += volts[i].sum;
        voltCount += volts[i].count;
      }
      System.out.printf("%-30s %9.1f%% %9.1fA %9.1fA %9.1fA %9.2fV %7.1fC %7.1fC%n",
          motor, 100.0 * active / Math.max(1, total),
          activeSum / Math.max(1, active),
          statorSum / Math.max(1, active),
          torqueSum / Math.max(1, torqueCount),
          voltSum / Math.max(1, voltCount),
          tempSum / Math.max(1, tempCount), tempPeak);
    }

    for (String motor : new TreeSet<>(supply.keySet())) {
      System.out.printf("%n%s%n", motor);
      System.out.printf("  %-10s %8s %10s %10s %10s %10s%n",
          "duty", "samples", "supply", "supply pk", "stator", "speed");
      for (int i = 0; i < 10; i++) {
        Draw amps = supply.get(motor)[i];
        if (amps.count == 0) {
          continue;
        }
        System.out.printf("  %.1f-%.1f    %8d %9.1fA %9.1fA %9.1fA %8.1frps%n",
            i / 10.0, (i + 1) / 10.0, amps.count, amps.mean(), amps.peak,
            stator.get(motor)[i].mean(), speed.get(motor)[i].mean());
      }
    }
  }

  /**
   * Finds episodes where a motor is commanded hard but has been dragged well
   * below its free speed, and reports how long they last, how long recovery
   * takes, and how fast the robot was driving through them. The question this
   * answers is whether the drivetrain outruns the mechanism.
   *
   * @param filter matched against the logged field name, e.g. "IntakeMotor0"
   */
  private static void reviewBog(String filter, List<File> logs) throws IOException {
    // Free speed is taken from the log rather than assumed, as the 95th
    // percentile of speed while commanded above 90% duty.
    List<Double> freeSpeeds = new ArrayList<>();
    List<Double> episodes = new ArrayList<>();
    double drivingWhileBogged = 0;
    long boggedSamples = 0;
    double drivingOverall = 0;
    long commandedSamples = 0;

    for (int pass = 0; pass < 2; pass++) {
      double freeSpeed = 0;
      if (pass == 1) {
        Collections.sort(freeSpeeds);
        if (freeSpeeds.isEmpty()) {
          System.out.println("no TalonFXIOData field matched " + filter);
          return;
        }
        freeSpeed = freeSpeeds.get((int) (freeSpeeds.size() * 0.95));
        System.out.printf("free speed (95th percentile at full duty): %.1f rps%n", freeSpeed);
      }

      for (File log : logs) {
        DataLogReader reader = new DataLogReader(log.getAbsolutePath());
        int motorEntry = -1;
        int velocityEntry = -1;
        int enabledEntry = -1;
        boolean enabled = false;
        double driveSpeed = 0;
        double bogStart = -1;
        double timestamp = 0;

        try {
          for (DataLogRecord record : reader) {
            if (record.isStart()) {
              var start = record.getStartData();
              if (start.name.equals(ENABLED)) {
                enabledEntry = start.entry;
              } else if (start.name.equals(FIELD_VELOCITY)) {
                velocityEntry = start.entry;
              } else if (start.type.equals("struct:TalonFXIOData") && start.name.contains(filter)) {
                motorEntry = start.entry;
              }
              continue;
            }
            if (record.isControl()) {
              continue;
            }
            if (record.getEntry() == enabledEntry) {
              enabled = record.getBoolean();
            } else if (record.getEntry() == velocityEntry) {
              ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
              driveSpeed = Math.hypot(buf.getDouble(0), buf.getDouble(Double.BYTES));
            } else if (enabled && record.getEntry() == motorEntry) {
              ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
              double duty = Math.abs(buf.getDouble(DUTY_CYCLE_OFFSET));
              double speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
              timestamp = record.getTimestamp() / 1e6;

              if (pass == 0) {
                if (duty > 0.9) {
                  freeSpeeds.add(speed);
                }
                continue;
              }
              if (duty < 0.5) {
                bogStart = -1;
                continue;
              }
              commandedSamples++;
              drivingOverall += driveSpeed;
              if (speed < 0.5 * freeSpeed) {
                boggedSamples++;
                drivingWhileBogged += driveSpeed;
                if (bogStart < 0) {
                  bogStart = timestamp;
                }
              } else if (bogStart >= 0) {
                episodes.add(timestamp - bogStart);
                bogStart = -1;
              }
            }
          }
        } catch (RuntimeException e) {
          // truncated log; keep what was read
        }
      }
    }

    System.out.printf("commanded above 50%% duty: %d samples, of which %d (%.1f%%) below half free speed%n",
        commandedSamples, boggedSamples, 100.0 * boggedSamples / Math.max(1, commandedSamples));
    System.out.printf("robot speed while commanded: %.2f m/s overall, %.2f m/s while bogged%n",
        drivingOverall / Math.max(1, commandedSamples),
        drivingWhileBogged / Math.max(1, boggedSamples));
    episodes.sort(Collections.reverseOrder());
    double totalTime = episodes.stream().mapToDouble(Double::doubleValue).sum();
    System.out.printf("%d bog episodes, %.2f s total, %.2f s mean, %.2f s longest%n",
        episodes.size(), totalTime, totalTime / Math.max(1, episodes.size()),
        episodes.isEmpty() ? 0 : episodes.get(0));
  }

  /**
   * Reports how well the flywheel holds its setpoint as battery voltage falls.
   * The flywheel is run in torque current control, whose achievable speed scales
   * with bus voltage, so a goal that is reachable on a fresh battery can become
   * unreachable on a sagging one no matter what the controller asks for.
   */
  private static void reviewShooter(List<File> logs) throws IOException {
    // Buckets of 0.5V from 8.0V up, index 0 meaning below 8.0V.
    Draw[] error = newBuckets();
    Draw[] atSetpoint = newBuckets();
    Draw[] goals = newBuckets();
    // What fraction would have been called ready at a tighter tolerance than the
    // 20 rps the robot uses. Debounce is ignored, so these are upper bounds.
    double[] tolerances = { 2, 3, 5, 10, 20 };
    Draw[][] within = new Draw[tolerances.length][];
    Arrays.setAll(within, i -> newBuckets());

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int goalEntry = -1;
      int atEntry = -1;
      int motorEntry = -1;
      int batteryEntry = -1;
      int enabledEntry = -1;
      boolean enabled = false;
      double goal = 0;
      double battery = 12;
      boolean at = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_AT_SETPOINT -> atEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case BATTERY_VOLTAGE -> batteryEntry = start.entry;
              case ENABLED -> enabledEntry = start.entry;
              default -> {
              }
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == goalEntry) {
            goal = record.getDouble();
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (entry == atEntry) {
            at = record.getBoolean();
          } else if (enabled && entry == motorEntry && goal > 1.0) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
            int bucket = Math.max(0, Math.min(9, (int) ((battery - 8.0) * 2) + 1));
            error[bucket].add(goal - speed, false);
            for (int t = 0; t < tolerances.length; t++) {
              within[t][bucket].add(Math.abs(goal - speed) < tolerances[t] ? 1 : 0, false);
            }
            atSetpoint[bucket].add(at ? 1 : 0, false);
            goals[bucket].add(goal, false);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("%-14s %10s %12s %12s %12s%n",
        "battery", "samples", "goal", "shortfall", "at setpoint");
    for (int i = 0; i < 10; i++) {
      if (error[i].count == 0) {
        continue;
      }
      String label = i == 0 ? "below 8.0V"
          : String.format("%.1f-%.1fV", 8.0 + (i - 1) * 0.5, 8.0 + i * 0.5);
      System.out.printf("%-14s %10d %11.1f %11.2f %11.1f%%%n",
          label, error[i].count, goals[i].mean(), error[i].mean(),
          100 * atSetpoint[i].mean());
    }

    System.out.printf("%n%-14s %s%n", "battery", "fraction within tolerance, rps");
    System.out.printf("%-14s", "");
    for (double tolerance : tolerances) {
      System.out.printf("%9.0f", tolerance);
    }
    System.out.println();
    for (int i = 0; i < 10; i++) {
      if (error[i].count == 0) {
        continue;
      }
      String label = i == 0 ? "below 8.0V"
          : String.format("%.1f-%.1fV", 8.0 + (i - 1) * 0.5, 8.0 + i * 0.5);
      System.out.printf("%-14s", label);
      for (int t = 0; t < tolerances.length; t++) {
        System.out.printf("%8.0f%%", 100 * within[t][i].mean());
      }
      System.out.println();
    }
  }

  /**
   * Prints one row per shot, keyed to match time so the moment can be found on
   * event video. The shortfall column is what the flywheel was actually missing
   * its goal by when the shot went out.
   */
  private static void reviewShots(List<File> logs) throws IOException {
    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int fireEntry = -1;
      int goalEntry = -1;
      int motorEntry = -1;
      int batteryEntry = -1;
      int matchTimeEntry = -1;
      int stateEntry = -1;
      int distanceEntry = -1;

      boolean firing = false;
      double goal = 0;
      double speed = 0;
      double battery = 12;
      double matchTime = Double.NaN;
      double distance = 0;
      String state = "?";
      int shots = 0;
      boolean printedHeader = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FIRE_SHOT -> fireEntry = start.entry;
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case BATTERY_VOLTAGE -> batteryEntry = start.entry;
              case MATCH_TIME -> matchTimeEntry = start.entry;
              case SHOOTER_STATE -> stateEntry = start.entry;
              case DISTANCE_TO_TARGET -> distanceEntry = start.entry;
              default -> {
              }
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == goalEntry) {
            goal = record.getDouble();
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (entry == matchTimeEntry) {
            matchTime = record.getDouble();
          } else if (entry == distanceEntry) {
            distance = record.getDouble();
          } else if (entry == stateEntry) {
            state = record.getString();
          } else if (entry == motorEntry) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
          } else if (entry == fireEntry) {
            boolean nowFiring = record.getBoolean();
            if (nowFiring && !firing) {
              if (!printedHeader) {
                System.out.printf("%n%s%n", log.getName());
                System.out.printf("  %-8s %8s %9s %8s %8s %10s  %s%n",
                    "match t", "dist", "goal", "actual", "short", "battery", "state");
                printedHeader = true;
              }
              shots++;
              System.out.printf("  %7.1fs %7.2fm %8.1f %7.1f %7.1f %9.2fV  %s%n",
                  matchTime, distance, goal, speed, goal - speed, battery, state);
            }
            firing = nowFiring;
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
      if (shots > 0) {
        System.out.printf("  %d shots%n", shots);
      }
    }
  }

  private static Draw[] newBuckets() {
    Draw[] buckets = new Draw[10];
    Arrays.setAll(buckets, i -> new Draw());
    return buckets;
  }

  /** Prints every field name in a log, so you can see what the code of the day recorded. */
  private static void listFields(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    TreeSet<String> names = new TreeSet<>();
    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          names.add(record.getStartData().name + "  [" + record.getStartData().type + "]");
        }
      }
    } catch (RuntimeException e) {
      // Log cut off mid-write; keep the field names read so far.
    }
    System.out.println(log.getName() + " - " + names.size() + " fields");
    names.forEach(n -> System.out.println("  " + n));
  }

  /**
   * Finds every falling edge of the autopilot Active flag and reports the robot's
   * field velocity at that moment. An alignment that ends while still moving is
   * one that declared itself finished on the way through the tolerance window.
   *
   * @return {totalEnds, endsWhileMoving}
   */
  private static int[] reviewAlignments(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());

    Map<Integer, String> names = new HashMap<>();
    int activeEntry = -1;
    int velocityEntry = -1;
    int poseEntry = -1;
    int targetEntry = -1;

    boolean lastActive = false;
    boolean seenActive = false;
    double lastSpeed = 0;
    double[] lastPose = null;
    double[] lastTarget = null;
    int ends = 0;
    int movingEnds = 0;
    int atTargetMovingEnds = 0;
    List<Double> speeds = new ArrayList<>();

    boolean truncated = false;
    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          names.put(start.entry, start.name);
          if (ACTIVE.equals(start.name)) {
            activeEntry = start.entry;
          } else if (FIELD_VELOCITY.equals(start.name)) {
            velocityEntry = start.entry;
          } else if (FIELD_POSE.equals(start.name)) {
            poseEntry = start.entry;
          } else if (TARGET.equals(start.name)) {
            targetEntry = start.entry;
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }

        int entry = record.getEntry();
        if (entry == velocityEntry) {
          lastSpeed = speedFromChassisSpeeds(record);
        } else if (entry == poseEntry) {
          lastPose = xy(record);
        } else if (entry == targetEntry) {
          lastTarget = xy(record);
        } else if (entry == activeEntry) {
          boolean active = record.getBoolean();
          if (seenActive && lastActive && !active) {
            ends++;
            speeds.add(lastSpeed);
            boolean moving = lastSpeed > 0.15;
            if (moving) {
              movingEnds++;
            }
            // Distinguish "drove through a waypoint on purpose" from "declared
            // itself finished at the target while still travelling".
            double distanceToTarget = (lastPose != null && lastTarget != null)
                ? Math.hypot(lastPose[0] - lastTarget[0], lastPose[1] - lastTarget[1])
                : Double.NaN;
            if (moving && distanceToTarget < 0.10) {
              atTargetMovingEnds++;
            }
            System.out.printf("    end: speed=%.2f m/s  distToTarget=%.2f m%n",
                lastSpeed, distanceToTarget);
          }
          lastActive = active;
          seenActive = true;
        }
      }
    } catch (RuntimeException e) {
      // A log cut off mid-write, e.g. the robot lost power. Keep what was read.
      truncated = true;
    }

    if (activeEntry < 0) {
      System.out.printf("%-52s no %s field (older code?)%n", log.getName(), ACTIVE);
      return new int[] { 0, 0 };
    }

    speeds.sort(Double::compare);
    System.out.printf("%-52s ends=%3d  moving=%3d  median=%.2f  max=%.2f m/s%s%n",
        log.getName(),
        ends,
        movingEnds,
        speeds.isEmpty() ? 0.0 : speeds.get(speeds.size() / 2),
        speeds.isEmpty() ? 0.0 : speeds.get(speeds.size() - 1),
        truncated ? "  (truncated)" : "");
    if (atTargetMovingEnds > 0) {
      System.out.printf("      ^ %d of these were AT the target (<10cm) while still moving%n",
          atTargetMovingEnds);
    }
    return new int[] { ends, movingEnds };
  }

  private static int bucketFor(double matchTime) {
    return matchTime > 100 ? 0 : matchTime > 75 ? 1 : matchTime > 50 ? 2 : matchTime > 25 ? 3 : 4;
  }

  /**
   * Reports how battery voltage behaves across a match, bucketed by time
   * remaining, so "we run out of battery near the end" can be checked rather than
   * assumed. Only samples taken while enabled are counted.
   */
  private static void reviewPower(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    int voltageEntry = -1;
    int brownedEntry = -1;
    int currentEntry = -1;
    int matchTimeEntry = -1;
    int enabledEntry = -1;

    double matchTime = Double.NaN;
    boolean enabled = false;
    double peakCurrent = 0;
    double minVoltage = Double.MAX_VALUE;
    int brownoutSamples = 0;
    int[] brownoutsByBucket = new int[5];
    double matchTimeAtMin = Double.NaN;
    // Count deep sags (below the 6.5V brownout threshold we configure)
    int deepSagSamples = 0;
    // Buckets: >100s, 100-75, 75-50, 50-25, <25s remaining
    double[] sums = new double[5];
    int[] counts = new int[5];

    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          switch (start.name) {
            case BATTERY_VOLTAGE -> voltageEntry = start.entry;
            case BROWNED_OUT -> brownedEntry = start.entry;
            case TOTAL_CURRENT -> currentEntry = start.entry;
            case MATCH_TIME -> matchTimeEntry = start.entry;
            case ENABLED -> enabledEntry = start.entry;
            default -> {
            }
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }
        int entry = record.getEntry();
        if (entry == matchTimeEntry) {
          matchTime = record.getDouble();
        } else if (entry == enabledEntry) {
          enabled = record.getBoolean();
        } else if (entry == brownedEntry) {
          if (enabled && record.getBoolean()) {
            brownoutSamples++;
            if (!Double.isNaN(matchTime)) {
              brownoutsByBucket[bucketFor(matchTime)]++;
            }
          }
        } else if (entry == currentEntry) {
          if (enabled) {
            peakCurrent = Math.max(peakCurrent, record.getDouble());
          }
        } else if (entry == voltageEntry) {
          double volts = record.getDouble();
          if (enabled && volts > 4.0 && !Double.isNaN(matchTime)) {
            if (volts < minVoltage) {
              minVoltage = volts;
              matchTimeAtMin = matchTime;
            }
            if (volts < 7.0) {
              deepSagSamples++;
            }
            int bucket = bucketFor(matchTime);
            sums[bucket] += volts;
            counts[bucket]++;
          }
        }
      }
    } catch (RuntimeException e) {
      // truncated log; keep what was read
    }

    if (voltageEntry < 0 || counts[0] + counts[1] + counts[2] + counts[3] + counts[4] == 0) {
      System.out.printf("%-42s no enabled battery samples%n", log.getName());
      return;
    }

    StringBuilder buckets = new StringBuilder();
    for (int i = 0; i < 5; i++) {
      buckets.append(counts[i] == 0 ? "   --  " : String.format(" %6.2f", sums[i] / counts[i]));
    }
    StringBuilder bo = new StringBuilder();
    for (int i = 0; i < 5; i++) {
      bo.append(String.format("%4d", brownoutsByBucket[i]));
    }
    System.out.printf("%-38s %s | min %.2f @t=%5.1f | sub7V %4d | brownouts%s | peak %4.0fA%n",
        log.getName().replace("akit_26-04-11_", "").replace(".wpilog", ""),
        buckets, minVoltage == Double.MAX_VALUE ? Double.NaN : minVoltage, matchTimeAtMin,
        deepSagSamples, bo, peakCurrent);
  }

  /**
   * Compares the accepted vision estimate against both the simulation's ground
   * truth pose (what SimVisionIO generates observations from) and the fused
   * estimator pose. Tells you whether a disagreement is vision being inaccurate
   * or the estimator having drifted.
   */
  private static void reviewVision(File log) throws IOException {
    DataLogReader reader = new DataLogReader(log.getAbsolutePath());
    int truthEntry = -1;
    int poseEntry = -1;
    int validatedEntry = -1;
    double[] truth = null;
    double[] pose = null;
    long truthTime = Long.MIN_VALUE;
    long poseTime = Long.MIN_VALUE;
    // WPILOG only records a value when it changes, so a stationary robot's pose
    // can be minutes stale. Only compare against a recently recorded sample.
    final long maxStaleUs = 100_000;
    int skipped = 0;
    List<Double> estimateVsTruth = new ArrayList<>();
    List<Double> estimateVsPose = new ArrayList<>();

    try {
      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          var start = record.getStartData();
          if (SIM_TRUTH.equals(start.name)) {
            truthEntry = start.entry;
          } else if (FIELD_POSE.equals(start.name)) {
            poseEntry = start.entry;
          } else if (VALIDATED.equals(start.name)) {
            validatedEntry = start.entry;
          }
          continue;
        }
        if (record.isControl()) {
          continue;
        }
        int entry = record.getEntry();
        if (entry == truthEntry) {
          truth = xy(record);
          truthTime = record.getTimestamp();
        } else if (entry == poseEntry) {
          pose = xy(record);
          poseTime = record.getTimestamp();
        } else if (entry == validatedEntry) {
          double[] estimate = xy(record);
          long now = record.getTimestamp();
          boolean truthFresh = truth != null && now - truthTime <= maxStaleUs;
          boolean poseFresh = pose != null && now - poseTime <= maxStaleUs;
          if (estimate != null && truthFresh) {
            estimateVsTruth.add(Math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]));
          } else {
            skipped++;
          }
          if (estimate != null && poseFresh) {
            estimateVsPose.add(Math.hypot(estimate[0] - pose[0], estimate[1] - pose[1]));
          }
        }
      }
    } catch (RuntimeException e) {
      // truncated log; keep what was read
    }

    if (validatedEntry < 0) {
      System.out.printf("%-46s no accepted vision estimates%n", log.getName());
      return;
    }
    System.out.printf("%-46s comparable=%d skipped(stale)=%d  vs truth: %.2fm  vs fused: %.2fm%n",
        log.getName(), estimateVsTruth.size(), skipped, median(estimateVsTruth), median(estimateVsPose));
  }

  private static double median(List<Double> values) {
    if (values.isEmpty()) {
      return Double.NaN;
    }
    List<Double> sorted = new ArrayList<>(values);
    sorted.sort(Double::compare);
    return sorted.get(sorted.size() / 2);
  }

  /** Pose2d serialises as x, y, theta little-endian doubles. */
  private static double[] xy(DataLogRecord record) {
    byte[] raw = record.getRaw();
    if (raw.length < 16) {
      return null;
    }
    ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
    return new double[] { buffer.getDouble(), buffer.getDouble() };
  }

  /** ChassisSpeeds serialises as three little-endian doubles: vx, vy, omega. */
  private static double speedFromChassisSpeeds(DataLogRecord record) {
    byte[] raw = record.getRaw();
    if (raw.length < 16) {
      return 0;
    }
    ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
    return Math.hypot(buffer.getDouble(), buffer.getDouble());
  }
}
