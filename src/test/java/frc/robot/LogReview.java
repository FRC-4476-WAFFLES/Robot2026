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
  private static final String FEEDER_MOTOR = "/Inputs/Indexer/FeederMotorData0";
  private static final String FEEDER_MOTOR_1 = "/Inputs/Indexer/FeederMotorData1";
  private static final String SPINDEXER_MOTOR = "/Inputs/Indexer/IndexerMotorData0";
  private static final String SPINDEXER_MOTOR_1 = "/Inputs/Indexer/IndexerMotorData1";
  private static final String FIRE_SHOT = "/RealOutputs/Commands/Fire shot";
  private static final String SHOOTER_STATE = "/RealOutputs/RobotState/Shooter State";
  private static final String DISTANCE_TO_TARGET = "/RealOutputs/Turret/Distance To Target";
  private static final String SHOOTING = "/RealOutputs/RobotState/Shooting";
  private static final String SHOOTER_HUB_COMMAND = "/RealOutputs/Commands/Shooter Hub";
  private static final String INTAKING = "/RealOutputs/RobotState/Intaking";
  private static final String AUTONOMOUS = "/DriverStation/Autonomous";
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
      case "gate" -> reviewGate(logs);
      case "modes" -> reviewModes(logs);
      case "recovery" -> reviewRecovery(logs);
      case "leadtime" -> reviewLeadTime(logs);
      case "battery" -> reviewBattery(logs);
      case "predict" -> reviewPredict(logs);
      case "energy" -> reviewEnergy(logs);
      case "ceiling" -> reviewCeiling(logs);
      case "brownout" -> reviewBrownout(logs);
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
  /**
   * The flywheel's share of the range error budget, in metres either side.
   * Matches {@code FlywheelConstants.ACCEPTABLE_RANGE_ERROR} so this analysis
   * scores shots the way the robot now gates them.
   */
  private static final double ACCEPTED_RANGE_ERROR = 0.35;
  /** Battery internal resistance, fitted from 63k logged samples in battery mode. */
  private static final double BATTERY_RESISTANCE = 0.0155;

  /**
   * The fastest the flywheel can turn at a given bus voltage, in rps.
   *
   * <p>
   * Measured, not derived. The old passing bug commanded 100.5 rps, a speed that
   * does not exist, so the wheel went flat out and the logs contain genuine
   * saturation points: 69 rps at 7.75 V, 83 at 8.75 V, 86 at 9.75 V, 88 at
   * 10.75 V. Below about 10 V it rises at 13.8 rps per volt; above that it
   * flattens against the wheel's own aerodynamic drag near 88 rps.
   *
   * <p>
   * What is left between this ceiling and the goal is the torque available to
   * recover from a ball, which is why the same dip costs 0.07 s on a good bus
   * and 0.59 s on a sagged one.
   */
  private static double speedCeiling(double battery) {
    return Math.min(88.0, 13.8 * (battery - 2.72));
  }

  /** Bus volts the drivetrain cap returns during a shot, from the predict mode. */
  private static final double DRIVE_CAP_VOLTS = 0.35;
  /** Speed tolerance used when measuring how long the flywheel takes to recover. */
  private static final double RECOVERY_TOLERANCE = 2.5;
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
    List<Double> recoveries = new ArrayList<>();
    List<Double> deficits = new ArrayList<>();
    List<double[]> hubShots = new ArrayList<>();
    List<Double> dips = new ArrayList<>();
    List<double[]> feederVsDip = new ArrayList<>();
    List<Double> dipDelays = new ArrayList<>();
    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int fireEntry = -1;
      int goalEntry = -1;
      int motorEntry = -1;
      int feederEntry = -1;
      double feederSpeed = 0;
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
      double[] pending = null;
      double[] recovering = null;
      double recoverFrom = 0;
      String pendingState = "?";
      double fireStart = 0;
      boolean printedHeader = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FIRE_SHOT -> fireEntry = start.entry;
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case FEEDER_MOTOR -> feederEntry = start.entry;
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
          } else if (entry == feederEntry) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            feederSpeed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
          } else if (entry == motorEntry) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
            if (pending != null) {
              // When the wheel first starts giving up speed. This is the window
              // a limit change has to land in if it is triggered by the fire
              // command itself.
              if (pending[9] < 0 && speed < pending[3] - 2.0) {
                pending[9] = record.getTimestamp() / 1e6 - fireStart;
                dipDelays.add(pending[9]);
              }
              pending[5] = Math.min(pending[5], speed);
              pending[6] = speed;
              pending[7] = record.getTimestamp() / 1e6 - fireStart;
            }
            if (recovering != null) {
              double elapsed = record.getTimestamp() / 1e6 - recoverFrom;
              if (Math.abs(recovering[2] - speed) < RECOVERY_TOLERANCE) {
                recoveries.add(elapsed);
                recovering = null;
              } else if (elapsed > 3.0) {
                // Never recovered within three seconds; count it at the cap.
                recoveries.add(3.0);
                recovering = null;
              }
            }
          } else if (entry == fireEntry) {
            boolean nowFiring = record.getBoolean();
            if (nowFiring && !firing) {
              fireStart = record.getTimestamp() / 1e6;
              if (!printedHeader) {
                System.out.printf("%n%s%n", log.getName());
                System.out.printf("  %-8s %8s %9s %8s %8s %8s %8s %9s  %s%n",
                    "match t", "dist", "goal", "at cmd", "min", "at end", "window", "battery", "state");
                printedHeader = true;
              }
              shots++;
              deficits.add(Math.abs(goal - speed));
              if (state.contains("HUB")) {
                hubShots.add(new double[] { distance, Math.abs(goal - speed), battery });
              }
              pending = new double[] { matchTime, distance, goal, speed, battery, speed, speed, 0, feederSpeed, -1 };
              pendingState = state;
            }
            if (firing && !nowFiring && pending != null) {
              // How far below the speed at the fire command the wheel was
              // dragged. This is what a readiness gate has to tolerate without
              // closing, since the ball causing it has already gone.
              dips.add(Math.max(0, pending[3] - pending[5]));
              feederVsDip.add(new double[] { pending[8], Math.max(0, pending[3] - pending[5]) });
              recovering = pending;
              recoverFrom = record.getTimestamp() / 1e6;
              // goal, speed at the command, min and last during the window
              System.out.printf("  %7.1fs %7.2fm %8.1f %7.1f %7.1f %7.1f %6.0fms %8.2fV  %s%n",
                  pending[0], pending[1], pending[2], pending[3], pending[5], pending[6],
                  pending[7] * 1000, pending[4], pendingState);
              pending = null;
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

    if (!deficits.isEmpty()) {
      Collections.sort(deficits);
      System.out.printf("%n%d shots, how far the flywheel was from its goal when the shot went out%n",
          deficits.size());
      for (double tolerance : new double[] { 1, 2, 2.5, 4, 10, 20 }) {
        long inside = deficits.stream().filter(d -> d < tolerance).count();
        System.out.printf("  within %4.1f rps: %3d of %d (%3.0f%%)%n",
            tolerance, inside, deficits.size(), 100.0 * inside / deficits.size());
      }
      System.out.printf("  median %.1f rps, 90th percentile %.1f rps%n",
          deficits.get(deficits.size() / 2), deficits.get(deficits.size() * 9 / 10));
    }

    if (!dipDelays.isEmpty()) {
      Collections.sort(dipDelays);
      System.out.printf("%ndelay from the fire command to the wheel starting to drop, %d shots%n",
          dipDelays.size());
      System.out.printf("  median %.3fs   25th %.3fs   10th %.3fs   shortest %.3fs%n",
          dipDelays.get(dipDelays.size() / 2), dipDelays.get(dipDelays.size() / 4),
          dipDelays.get(dipDelays.size() / 10), dipDelays.get(0));
      for (double bound : new double[] { 0.05, 0.10, 0.20 }) {
        long under = dipDelays.stream().filter(d -> d < bound).count();
        System.out.printf("  %d of %d (%.0f%%) dropped within %.2fs of the command%n",
            under, dipDelays.size(), 100.0 * under / dipDelays.size(), bound);
      }
    }

    if (!feederVsDip.isEmpty()) {
      // Does a faster feeder mean a smaller flywheel dip? A ball entering with
      // more speed of its own takes less energy out of the wheel, since the wheel
      // only has to supply the difference.
      System.out.printf("%nflywheel dip against feeder speed at the shot%n");
      System.out.printf("  %-16s %8s %14s%n", "feeder speed", "shots", "median dip");
      for (double low = 0; low < 45; low += 5) {
        final double lo = low;
        List<Double> inBucket = new ArrayList<>(feederVsDip.stream()
            .filter(shot -> shot[0] >= lo && shot[0] < lo + 5)
            .map(shot -> shot[1]).toList());
        if (inBucket.size() < 5) {
          continue;
        }
        Collections.sort(inBucket);
        System.out.printf("  %4.0f-%4.0f rps    %8d %11.1f rps%n",
            low, low + 5, inBucket.size(), inBucket.get(inBucket.size() / 2));
      }
    }

    if (!dips.isEmpty()) {
      Collections.sort(dips);
      System.out.printf("%nhow far the wheel drops during a shot, %d shots%n", dips.size());
      for (double bound : new double[] { 2, 4, 6, 10, 15, 25 }) {
        long inside = dips.stream().filter(d -> d < bound).count();
        System.out.printf("  drop under %4.1f rps: %3d of %d (%3.0f%%)%n",
            bound, inside, dips.size(), 100.0 * inside / dips.size());
      }
      System.out.printf("  median %.1f rps, 75th %.1f rps, 90th %.1f rps, worst %.1f rps%n",
          dips.get(dips.size() / 2), dips.get(dips.size() * 3 / 4),
          dips.get(dips.size() * 9 / 10), dips.get(dips.size() - 1));
    }

    if (!hubShots.isEmpty()) {
      hubShots.sort((a, b) -> Double.compare(a[0], b[0]));
      System.out.printf("%n%d hub shots, from %.2fm to %.2fm%n",
          hubShots.size(), hubShots.get(0)[0], hubShots.get(hubShots.size() - 1)[0]);
      System.out.printf("  %-12s %8s %12s %14s %10s %10s %11s%n",
          "distance", "shots", "med deficit", "within 4 rps", "scaled", "within", "med battery");
      for (double low = 1.0; low < 7.0; low += 0.5) {
        final double lo = low;
        List<double[]> inBucket = hubShots.stream()
            .filter(shot -> shot[0] >= lo && shot[0] < lo + 0.5).toList();
        if (inBucket.isEmpty()) {
          continue;
        }
        List<Double> bucketDeficits = new ArrayList<>(inBucket.stream().map(shot -> shot[1]).toList());
        List<Double> batteries = new ArrayList<>(inBucket.stream().map(shot -> shot[2]).toList());
        Collections.sort(bucketDeficits);
        Collections.sort(batteries);
        // A fixed rps tolerance is the wrong yardstick: range error is
        // v * dv / (2 * R), so the same speed error costs far less range up
        // close than it does far out. Score each shot against the speed error
        // that would move it by ACCEPTED_RANGE_ERROR at its own distance.
        double distance = lo + 0.25;
        double speed = 45 + 5 * distance;
        double scaled = Math.max(2.0,
            Math.min(8.0, speed * ACCEPTED_RANGE_ERROR / (2 * distance)));
        long fixed = bucketDeficits.stream().filter(d -> d < 4.0).count();
        long good = bucketDeficits.stream().filter(d -> d < scaled).count();
        System.out.printf("  %4.1f-%4.1fm %8d %11.1f %12.0f%% %10.1f %9.0f%% %10.2fV%n",
            low, low + 0.5, inBucket.size(), bucketDeficits.get(bucketDeficits.size() / 2),
            100.0 * fixed / inBucket.size(), scaled,
            100.0 * good / inBucket.size(), batteries.get(batteries.size() / 2));
      }
    }

    if (!recoveries.isEmpty()) {
      Collections.sort(recoveries);
      System.out.printf("%nrecovery to within %.1f rps of goal after a shot, %d samples%n",
          RECOVERY_TOLERANCE, recoveries.size());
      System.out.printf("  median %.2fs   75th %.2fs   90th %.2fs   worst %.2fs%n",
          recoveries.get(recoveries.size() / 2),
          recoveries.get(recoveries.size() * 3 / 4),
          recoveries.get(recoveries.size() * 9 / 10),
          recoveries.get(recoveries.size() - 1));
      long instant = recoveries.stream().filter(r -> r < 0.05).count();
      long never = recoveries.stream().filter(r -> r >= 3.0).count();
      System.out.printf("  %d of %d (%.0f%%) were already within tolerance when firing stopped%n",
          instant, recoveries.size(), 100.0 * instant / recoveries.size());
      System.out.printf("  %d of %d (%.0f%%) never recovered within 3s -- the wheel could not reach its goal%n",
          never, recoveries.size(), 100.0 * never / recoveries.size());
    }
  }

  /**
   * Replays a proposed flywheel readiness gate over the logs and counts how often
   * it would have opened and closed, to see whether a tighter tolerance would
   * make the shooter chatter. Only samples where a shot is actually wanted count.
   *
   * <p>
   * The debounce matches {@code Flywheel}: the gate must hold true continuously
   * for 0.25 s before it opens, and closes the instant the wheel falls outside
   * tolerance.
   */
  private static void reviewGate(List<File> logs) throws IOException {
    // The proposed gate: a tolerance that scales with distance, because the same
    // speed error costs far less range up close, plus a falling debounce so the
    // dip a ball causes on its way out does not shut the gate behind it.
    final double fallingDebounce = 0.25;
    final double risingDebounce = 0.25;
    final double minTolerance = 2.0;
    final double maxTolerance = 8.0;

    // Buckets of 0.5m from 1.0m, run twice: once against the wheel as it
    // actually behaved, and once with the deficit shrunk by however much faster
    // the wheel would have recovered under the new current limits and the higher
    // bus voltage the drivetrain cap buys.
    int buckets = 10;
    double[][] openTime = new double[2][buckets];
    double[][] wantedTime = new double[2][buckets];
    double[] toleranceSum = new double[buckets];
    int[][] opens = new int[2][buckets];

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int goalEntry = -1;
      int motorEntry = -1;
      int enabledEntry = -1;
      int shootingEntry = -1;
      int distanceEntry = -1;
      boolean enabled = false;
      boolean shooting = false;
      double goal = 0;
      double distance = 0;
      double[] trueSince = { -1, -1 };
      double[] outsideSince = { -1, -1 };
      boolean[] open = { false, false };
      double last = -1;
      int batteryEntry = -1;
      double battery = 12;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case ENABLED -> enabledEntry = start.entry;
              case SHOOTING -> shootingEntry = start.entry;
              case DISTANCE_TO_TARGET -> distanceEntry = start.entry;
              case BATTERY_VOLTAGE -> batteryEntry = start.entry;
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
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (entry == shootingEntry) {
            shooting = record.getBoolean();
          } else if (entry == goalEntry) {
            goal = record.getDouble();
          } else if (entry == distanceEntry) {
            distance = record.getDouble();
          } else if (entry == motorEntry) {
            double now = record.getTimestamp() / 1e6;
            double step = last < 0 ? 0 : Math.min(0.1, now - last);
            last = now;
            if (!enabled || !shooting || goal <= 1.0 || distance < 1.0) {
              Arrays.fill(trueSince, -1);
              Arrays.fill(outsideSince, -1);
              Arrays.fill(open, false);
              continue;
            }
            int bucket = Math.min(buckets - 1, (int) ((distance - 1.0) * 2));
            double tolerance = Math.max(minTolerance,
                Math.min(maxTolerance, goal * ACCEPTED_RANGE_ERROR / (2 * distance)));
            toleranceSum[bucket] += tolerance * step;

            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
            double deficit = Math.abs(goal - speed);

            // How much faster the wheel would come back. Available recovery
            // current is what is left of the bus above back-EMF, so both the
            // raised ceiling and the volts the drivetrain cap returns show up
            // here. A residual deficit shrinks in proportion.
            double headroomNow = Math.max(0.5, speedCeiling(battery) - speed);
            double headroomNew = Math.max(0.5, speedCeiling(battery + DRIVE_CAP_VOLTS) - speed);
            double improvedDeficit = deficit * headroomNow / headroomNew;

            for (int variant = 0; variant < 2; variant++) {
              wantedTime[variant][bucket] += step;
              if ((variant == 0 ? deficit : improvedDeficit) < tolerance) {
                outsideSince[variant] = -1;
                if (trueSince[variant] < 0) {
                  trueSince[variant] = now;
                }
                if (!open[variant] && now - trueSince[variant] >= risingDebounce) {
                  open[variant] = true;
                  opens[variant][bucket]++;
                }
              } else {
                trueSince[variant] = -1;
                if (outsideSince[variant] < 0) {
                  outsideSince[variant] = now;
                }
                if (open[variant] && now - outsideSince[variant] >= fallingDebounce) {
                  open[variant] = false;
                }
              }
              if (open[variant]) {
                openTime[variant][bucket] += step;
              }
            }
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("gate open by distance: tolerance = goal * %.2fm / (2 * distance), "
        + "clamped %.0f-%.0f rps, falling debounce %.2fs%n%n",
        ACCEPTED_RANGE_ERROR, minTolerance, maxTolerance, fallingDebounce);
    System.out.printf("  %-12s %10s %12s %14s %16s%n",
        "distance", "time spent", "tolerance", "open as logged", "open with power mgr");
    for (int i = 0; i < buckets; i++) {
      if (wantedTime[0][i] < 1.0) {
        continue;
      }
      System.out.printf("  %4.1f-%4.1fm %9.0fs %10.1f rps %12.0f%% %15.0f%%%n",
          1.0 + i * 0.5, 1.5 + i * 0.5, wantedTime[0][i], toleranceSum[i] / wantedTime[0][i],
          100 * openTime[0][i] / wantedTime[0][i], 100 * openTime[1][i] / wantedTime[1][i]);
    }
  }

  /**
   * Splits every managed motor's supply current by what the robot was doing, so a
   * budget can be set from what a mechanism actually needs in each mode rather
   * than from its worst case across the whole match.
   */
  private static void reviewModes(List<File> logs) throws IOException {
    // mode index: 0 auto, 1 teleop shooting, 2 teleop intaking, 3 teleop both, 4
    // teleop neither
    String[] modeNames = { "auto", "shooting", "intaking", "shoot+intake", "idle" };
    Map<String, Draw[]> draws = new HashMap<>();

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> talons = new HashMap<>();
      int enabledEntry = -1;
      int autoEntry = -1;
      int shootEntry = -1;
      int intakeEntry = -1;
      boolean enabled = false;
      boolean auto = false;
      boolean shooting = false;
      boolean intaking = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.name.equals(AUTONOMOUS)) {
              autoEntry = start.entry;
            } else if (start.name.equals(SHOOTING)) {
              shootEntry = start.entry;
            } else if (start.name.equals(INTAKING)) {
              intakeEntry = start.entry;
            } else if (start.type.equals("struct:TalonFXIOData")) {
              talons.put(start.entry, shortName(start.name));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == autoEntry) {
            auto = record.getBoolean();
          } else if (entry == shootEntry) {
            shooting = record.getBoolean();
          } else if (entry == intakeEntry) {
            intaking = record.getBoolean();
          } else if (enabled && talons.containsKey(entry)) {
            int mode = auto ? 0 : shooting && intaking ? 3 : shooting ? 1 : intaking ? 2 : 4;
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            draws.computeIfAbsent(talons.get(entry), k -> newBuckets())[mode]
                .add(Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET)), false);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("supply current by mode, mean over samples with peak in brackets%n%n");
    System.out.printf("%-30s", "motor");
    for (String mode : modeNames) {
      System.out.printf("%18s", mode);
    }
    System.out.println();
    for (String motor : new TreeSet<>(draws.keySet())) {
      Draw[] modes = draws.get(motor);
      System.out.printf("%-30s", motor);
      for (int i = 0; i < modeNames.length; i++) {
        System.out.printf("%18s", modes[i].count == 0 ? "-"
            : String.format("%.1f (%.0f)", modes[i].mean(), modes[i].peak));
      }
      System.out.println();
    }
  }

  /**
   * Works out whether the new current limits would let the flywheel recover fast
   * enough to shoot, by identifying the wheel's real acceleration per amp from
   * the logs and then re-running each measured dip under the new ceilings.
   *
   * <p>
   * The identification is the honest part: for every stretch where the wheel is
   * accelerating back towards its goal, {@code alpha / current} gives
   * {@code torqueConstant / inertia} directly, with no need to know either. What
   * follows from it is an estimate, and it assumes the battery holds up.
   */
  private static void reviewRecovery(List<File> logs) throws IOException {
    // Identified from the logs below.
    List<double[]> accelSamples = new ArrayList<>();
    // Each measured dip: how deep, and how long it took to come back.
    List<double[]> dips = new ArrayList<>();

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int goalEntry = -1;
      int motorEntry = -1;
      int enabledEntry = -1;
      boolean enabled = false;
      double goal = 0;
      double lastSpeed = Double.NaN;
      double lastTime = Double.NaN;
      double dipStart = -1;
      double dipDepth = 0;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case ENABLED -> enabledEntry = start.entry;
              default -> {
              }
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          if (record.getEntry() == enabledEntry) {
            enabled = record.getBoolean();
          } else if (record.getEntry() == goalEntry) {
            goal = record.getDouble();
          } else if (enabled && record.getEntry() == motorEntry) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
            double stator = Math.abs(buf.getDouble(STATOR_CURRENT_OFFSET));
            double now = record.getTimestamp() / 1e6;

            if (!Double.isNaN(lastSpeed) && now - lastTime > 0.005 && now - lastTime < 0.06) {
              double alpha = (speed - lastSpeed) / (now - lastTime);
              // Only stretches that are genuinely accelerating under real
              // current, and below goal so the controller is asking for all of
              // it rather than holding station.
              if (alpha > 20 && stator > 20 && goal > 1 && speed < goal - 2) {
                accelSamples.add(new double[] { alpha, stator, speed });
              }
            }

            if (goal > 1) {
              double error = goal - speed;
              if (error > 3 && dipStart < 0) {
                dipStart = now;
                dipDepth = error;
              } else if (dipStart >= 0) {
                dipDepth = Math.max(dipDepth, error);
                if (error < 2) {
                  dips.add(new double[] { dipDepth, now - dipStart, goal });
                  dipStart = -1;
                } else if (now - dipStart > 3.0) {
                  dipStart = -1;
                }
              }
            }
            lastSpeed = speed;
            lastTime = now;
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    if (accelSamples.isEmpty() || dips.isEmpty()) {
      System.out.println("not enough recovery data in these logs");
      return;
    }

    // rps per second, per amp of stator current
    List<Double> ratios = new ArrayList<>(
        accelSamples.stream().map(sample -> sample[0] / sample[1]).sorted().toList());
    double accelPerAmp = ratios.get(ratios.size() / 2);
    System.out.printf("identified from %d accelerating samples: %.3f rps/s per amp of stator%n",
        accelSamples.size(), accelPerAmp);
    System.out.printf("  so 75A gives %.0f rps/s, 160A gives %.0f rps/s%n%n",
        75 * accelPerAmp, 160 * accelPerAmp);

    // Today the 60A supply limit binds first. At high duty supply is close to
    // stator, so the effective stator ceiling is near the supply limit.
    double effectiveNow = 75;
    double effectiveNew = 160;

    System.out.printf("%d measured dips. Recovery time to come back within 2 rps.%n", dips.size());
    System.out.printf("The model column is what the identified acceleration predicts at today's%n"
        + "effective ceiling. Where measured runs far above it, the wheel was not getting%n"
        + "the current the limit allows -- it had no voltage left to push it, and raising%n"
        + "the limit cannot help those.%n%n");
    System.out.printf("  %-14s %8s %12s %12s %14s%n",
        "dip depth", "dips", "measured", "model now", "model at 160A");
    for (double low = 3; low < 30; low += 5) {
      final double lo = low;
      List<double[]> inBucket = dips.stream()
          .filter(dip -> dip[0] >= lo && dip[0] < lo + 5).toList();
      if (inBucket.size() < 5) {
        continue;
      }
      List<Double> times = new ArrayList<>(inBucket.stream().map(dip -> dip[1]).sorted().toList());
      double measured = times.get(times.size() / 2);
      double depth = inBucket.stream().mapToDouble(dip -> dip[0]).average().orElse(0);
      double predictedNow = depth / (effectiveNow * accelPerAmp);
      double predictedNew = depth / (effectiveNew * accelPerAmp);
      System.out.printf("  %4.0f-%4.0f rps %8d %11.2fs %11.2fs %13.2fs%s%n",
          low, low + 5, inBucket.size(), measured, predictedNow, predictedNew,
          measured > predictedNow * 2 ? "   <- voltage limited, not current limited" : "");
    }
  }

  /**
   * Answers when the drivetrain limit would need to be applied to be any use.
   *
   * <p>
   * Two numbers decide it. How long after the robot decides it is shooting the
   * first ball actually leaves, which is the lead time available for a blocking
   * CAN write to land. And how much the drivetrain is drawing in the moment
   * before a shot, because a cap on a drivetrain that is already idle saves
   * nothing.
   */
  private static void reviewLeadTime(List<File> logs) throws IOException {
    // Each candidate trigger, and how long before the ball leaves it fires.
    Map<String, List<Double>> leads = new java.util.LinkedHashMap<>();
    leads.put("RobotState/Shooting", new ArrayList<>());
    leads.put("flywheel goal set", new ArrayList<>());
    leads.put("Commands/Shooter Hub", new ArrayList<>());
    leads.put("flywheel within 10 rps", new ArrayList<>());
    leads.put("flywheel within 4 rps", new ArrayList<>());
    leads.put("autopilot aligning", new ArrayList<>());
    List<Double> driveBeforeShot = new ArrayList<>();

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> driveSupply = new HashMap<>();
      int shootEntry = -1;
      int goalEntry = -1;
      int hubEntry = -1;
      int motorEntry = -1;
      int activeEntry = -1;
      int fireEntry = -1;
      double goal = 0;
      int enabledEntry = -1;
      int batteryEntry = -1;
      boolean enabled = false;
      boolean shooting = false;
      boolean firing = false;
      double battery = 12;
      Map<String, Double> since = new HashMap<>();
      boolean countedThisBurst = false;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(SHOOTING)) {
              shootEntry = start.entry;
            } else if (start.name.equals(FLYWHEEL_GOAL)) {
              goalEntry = start.entry;
            } else if (start.name.equals(SHOOTER_HUB_COMMAND)) {
              hubEntry = start.entry;
            } else if (start.name.equals(FLYWHEEL_MOTOR)) {
              motorEntry = start.entry;
            } else if (start.name.equals(ACTIVE)) {
              activeEntry = start.entry;
            } else if (start.name.equals(FIRE_SHOT)) {
              fireEntry = start.entry;
            } else if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.name.equals(BATTERY_VOLTAGE)) {
              batteryEntry = start.entry;
            } else if (start.name.endsWith("DriveCurrentAmps")) {
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("DriveAppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          double now = record.getTimestamp() / 1e6;
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (entry == shootEntry) {
            boolean nowShooting = record.getBoolean();
            if (nowShooting && !shooting) {
              since.put("RobotState/Shooting", now);
              countedThisBurst = false;
            } else if (!nowShooting) {
              since.remove("RobotState/Shooting");
            }
            shooting = nowShooting;
          } else if (entry == goalEntry) {
            goal = record.getDouble();
            if (goal > 1.0) {
              since.putIfAbsent("flywheel goal set", now);
            } else {
              since.remove("flywheel goal set");
              since.remove("flywheel within 10 rps");
              since.remove("flywheel within 4 rps");
            }
          } else if (entry == activeEntry) {
            if (record.getBoolean()) {
              since.putIfAbsent("autopilot aligning", now);
            } else {
              since.remove("autopilot aligning");
            }
          } else if (entry == motorEntry && goal > 1.0) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double error = Math.abs(goal - Math.abs(buf.getDouble(VELOCITY_OFFSET)));
            if (error < 10) {
              since.putIfAbsent("flywheel within 10 rps", now);
            } else {
              since.remove("flywheel within 10 rps");
            }
            if (error < 4) {
              since.putIfAbsent("flywheel within 4 rps", now);
            } else {
              since.remove("flywheel within 4 rps");
            }
          } else if (entry == hubEntry) {
            if (record.getBoolean()) {
              since.putIfAbsent("Commands/Shooter Hub", now);
            } else {
              since.remove("Commands/Shooter Hub");
            }
          } else if (entry == fireEntry) {
            boolean nowFiring = record.getBoolean();
            if (nowFiring && !firing && enabled) {
              if (!countedThisBurst) {
                for (var trigger : leads.entrySet()) {
                  Double start = since.get(trigger.getKey());
                  if (start != null) {
                    trigger.getValue().add(now - start);
                  }
                }
                countedThisBurst = true;
              }
              double total = driveSupply.values().stream().mapToDouble(Double::doubleValue).sum();
              driveBeforeShot.add(total);
            }
            firing = nowFiring;
          } else if (statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          } else if (appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("how much warning each candidate trigger gives before the ball leaves%n%n");
    System.out.printf("  %-24s %8s %9s %9s %9s %14s%n",
        "trigger", "bursts", "median", "25th", "10th", "under 0.3s");
    for (var trigger : leads.entrySet()) {
      List<Double> times = trigger.getValue();
      if (times.isEmpty()) {
        System.out.printf("  %-24s %8s%n", trigger.getKey(), "never set");
        continue;
      }
      Collections.sort(times);
      long tooShort = times.stream().filter(t -> t < 0.3).count();
      System.out.printf("  %-24s %8d %8.2fs %8.2fs %8.2fs %12.0f%%%n",
          trigger.getKey(), times.size(), times.get(times.size() / 2),
          times.get(times.size() / 4), times.get(times.size() / 10),
          100.0 * tooShort / times.size());
    }
    System.out.println();

    Collections.sort(driveBeforeShot);
    System.out.printf("estimated drivetrain supply current at the moment of a shot, %d shots%n",
        driveBeforeShot.size());
    System.out.printf("  median %.0fA   75th %.0fA   90th %.0fA   peak %.0fA   (all four motors)%n",
        driveBeforeShot.get(driveBeforeShot.size() / 2),
        driveBeforeShot.get(driveBeforeShot.size() * 3 / 4),
        driveBeforeShot.get(driveBeforeShot.size() * 9 / 10),
        driveBeforeShot.get(driveBeforeShot.size() - 1));
    for (double bound : new double[] { 40, 60, 100 }) {
      long over = driveBeforeShot.stream().filter(d -> d > bound).count();
      System.out.printf("  %d of %d (%.0f%%) were drawing more than %.0fA%n",
          over, driveBeforeShot.size(), 100.0 * over / driveBeforeShot.size(), bound);
    }
  }

  private static void updateDriveSupply(String base, Map<String, Double> stator,
      Map<String, Double> volts, double battery, Map<String, Double> driveSupply) {
    Double amps = stator.get(base);
    Double applied = volts.get(base);
    if (amps == null || applied == null || battery < 4.0) {
      return;
    }
    driveSupply.put(base, amps * applied / battery);
  }

  /**
   * Identifies the battery's internal resistance from the logs, then predicts
   * what the bus would have done with the drivetrain capped.
   *
   * <p>
   * A battery under load reads {@code openCircuit - current * resistance}, so
   * regressing every logged voltage against the total current at that moment
   * recovers both numbers. Capping the drivetrain removes current, and the
   * voltage that comes back is that removed current times the resistance. That
   * is the entire benefit of the drivetrain cap, and this is the only way to size
   * it without a robot.
   */
  private static void reviewBattery(List<File> logs) throws IOException {
    long n = 0;
    double sumI = 0;
    double sumV = 0;
    double sumII = 0;
    double sumIV = 0;
    // Voltage and estimated drivetrain draw at the moment of each shot.
    List<double[]> shots = new ArrayList<>();

    System.out.printf("internal resistance per log. A pack whose resistance is high sags harder%n"
        + "at the same current than any amount of current limiting can make up for.%n%n");
    System.out.printf("  %-28s %10s %14s %12s%n", "log", "samples", "resistance", "open circuit");

    for (File log : logs) {
      long logN = 0;
      double logI = 0;
      double logV = 0;
      double logII = 0;
      double logIV = 0;
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> driveSupply = new HashMap<>();
      int voltageEntry = -1;
      int currentEntry = -1;
      int enabledEntry = -1;
      int fireEntry = -1;
      boolean enabled = false;
      boolean firing = false;
      double battery = 12;
      double total = 0;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(BATTERY_VOLTAGE)) {
              voltageEntry = start.entry;
            } else if (start.name.equals(TOTAL_CURRENT)) {
              currentEntry = start.entry;
            } else if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.name.equals(FIRE_SHOT)) {
              fireEntry = start.entry;
            } else if (start.name.endsWith("DriveCurrentAmps")) {
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("DriveAppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == currentEntry) {
            total = record.getDouble();
          } else if (entry == voltageEntry) {
            battery = record.getDouble();
            // Skip the impossible readings the PDH produces during its stuck
            // channel fault, which would drag the fit badly.
            if (enabled && battery > 4.0 && total > 0 && total < 260) {
              n++;
              sumI += total;
              sumV += battery;
              sumII += total * total;
              sumIV += total * battery;
              logN++;
              logI += total;
              logV += battery;
              logII += total * total;
              logIV += total * battery;
            }
          } else if (entry == fireEntry) {
            boolean nowFiring = record.getBoolean();
            if (nowFiring && !firing && enabled) {
              shots.add(new double[] {
                  battery, driveSupply.values().stream().mapToDouble(Double::doubleValue).sum() });
            }
            firing = nowFiring;
          } else if (statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          } else if (appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }

      if (logN > 2000) {
        double logSlope = (logN * logIV - logI * logV) / (logN * logII - logI * logI);
        System.out.printf("  %-28s %10d %11.1f mOhm %10.2fV%n",
            log.getName().replace("akit_26-04-11_", "").replace(".wpilog", "").replace("_onwel", ""),
            logN, -logSlope * 1000, (logV - logSlope * logI) / logN);
      }
    }
    System.out.println();

    if (n < 100 || shots.isEmpty()) {
      System.out.println("not enough data");
      return;
    }
    double slope = (n * sumIV - sumI * sumV) / (n * sumII - sumI * sumI);
    double openCircuit = (sumV - slope * sumI) / n;
    double resistance = -slope;
    System.out.printf("fitted from %d samples: open circuit %.2fV, internal resistance %.1f mOhm%n",
        n, openCircuit, resistance * 1000);
    System.out.printf("  so every 100A of draw costs %.2fV of bus%n%n", 100 * resistance);

    for (double cap : new double[] { 20, 10 }) {
      List<Double> gains = new ArrayList<>();
      List<Double> after = new ArrayList<>();
      for (double[] shot : shots) {
        double saved = Math.max(0, shot[1] - 4 * cap);
        gains.add(saved * resistance);
        after.add(shot[0] + saved * resistance);
      }
      Collections.sort(gains);
      Collections.sort(after);
      long helped = gains.stream().filter(g -> g > 0.25).count();
      System.out.printf("drivetrain capped at %.0fA per motor, over %d shots:%n", cap, shots.size());
      System.out.printf("  voltage recovered: median %.2fV, 75th %.2fV, 90th %.2fV, best %.2fV%n",
          gains.get(gains.size() / 2), gains.get(gains.size() * 3 / 4),
          gains.get(gains.size() * 9 / 10), gains.get(gains.size() - 1));
      System.out.printf("  %d of %d shots (%.0f%%) gain more than 0.25V%n",
          helped, shots.size(), 100.0 * helped / shots.size());
      long lowBefore = shots.stream().filter(shot -> shot[0] < 9.0).count();
      long lowAfter = after.stream().filter(v -> v < 9.0).count();
      System.out.printf("  shots taken below 9V: %d before, %d after%n%n", lowBefore, lowAfter);
    }
  }

  /**
   * Estimates how many more shots would land with the drivetrain capped.
   *
   * <p>
   * The chain is: capping the drivetrain removes current, removed current times
   * the battery's 15.5 mOhm gives volts back, and the volts a motor has above its
   * back-EMF is what it can turn into recovery current. A shot fired with the
   * wheel a given distance below its goal would, with proportionally more
   * recovery current, have been fired that much closer to it.
   *
   * <p>
   * The assumptions, all of them: back-EMF is linear in wheel speed at the
   * gradient the logs imply, recovery is proportional to available current, and
   * the driver would have taken the same shot at the same moment. The last one is
   * the weakest — a real driver whose robot accelerates less would drive
   * differently. Treat the answer as an order of magnitude.
   */
  private static void reviewPredict(List<File> logs) throws IOException {

    List<double[]> shots = collectShots(logs);

    // Each candidate: drivetrain cap, feeder cap per motor, spindexer cap per
    // motor. The drivetrain rows are what is already built; the rest ask what
    // capping the ball path on top of it would add.
    double[][] candidates = {
        { 45, 99, 99 }, { 20, 99, 99 }, { 10, 99, 99 },
        { 10, 20, 20 }, { 5, 20, 20 }, { 0, 20, 20 },
        { 10, 15, 15 }, { 10, 10, 10 }, { 10, 6, 6 },
        // The ceiling: nothing else on the robot drawing anything at all.
        { 0, 0, 0 },
    };

    System.out.printf("%8s %8s %10s %14s %12s %12s%n",
        "drive", "feeder", "spindexer", "volts back", "land", "vs today");
    int baseline = -1;
    for (double[] candidate : candidates) {
      int landed = 0;
      int total = 0;
      double gainSum = 0;
      for (double[] shot : shots) {
        double distance = shot[0];
        double goal = shot[1];
        double speed = shot[2];
        double battery = shot[3];
        if (distance < 1.0 || goal < 1.0 || battery < 4.0) {
          continue;
        }
        total++;
        double saved = Math.max(0, shot[4] - 4 * candidate[0])
            + Math.max(0, shot[5] - 2 * candidate[1])
            + Math.max(0, shot[6] - 2 * candidate[2]);
        double recovered = saved * BATTERY_RESISTANCE;
        gainSum += recovered;
        double tolerance = Math.max(2.0,
            Math.min(8.0, goal * ACCEPTED_RANGE_ERROR / (2 * distance)));
        double deficit = Math.abs(goal - speed);
        // Recovery rate scales with how far the wheel is below the fastest it
        // could turn on this bus. More volts raises that ceiling, so a residual
        // deficit shrinks in proportion.
        double headroomNow = Math.max(0.5, speedCeiling(battery) - speed);
        double headroomNew = Math.max(0.5, speedCeiling(battery + recovered) - speed);
        double improved = deficit * headroomNow / headroomNew;
        if (improved < tolerance) {
          landed++;
        }
      }
      if (total == 0) {
        System.out.println("no shots");
        return;
      }
      if (baseline < 0) {
        baseline = landed;
      }
      System.out.printf("%8.0f %8s %10s %11.2fV %8d %3.0f%% %8s%n",
          candidate[0],
          candidate[1] > 90 ? "-" : String.format("%.0f", candidate[1]),
          candidate[2] > 90 ? "-" : String.format("%.0f", candidate[2]),
          gainSum / total, landed, 100.0 * landed / total,
          landed == baseline ? "-" : String.format("%+d", landed - baseline));
    }
  }

  /**
   * {distance, goal, speed, battery, drivetrain supply, feeder supply, spindexer
   * supply} at the moment of each shot. The last two are measured directly, the
   * drivetrain one estimated from stator current and applied voltage.
   */
  private static List<double[]> collectShots(List<File> logs) throws IOException {
    List<double[]> shots = new ArrayList<>();
    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> driveSupply = new HashMap<>();
      int goalEntry = -1;
      int motorEntry = -1;
      int batteryEntry = -1;
      int enabledEntry = -1;
      int fireEntry = -1;
      int distanceEntry = -1;
      int[] ballPathEntries = { -1, -1, -1, -1 };
      double[] ballPathDraw = new double[4];
      boolean enabled = false;
      boolean firing = false;
      double goal = 0;
      double speed = 0;
      double battery = 12;
      double distance = 0;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
              case FLYWHEEL_MOTOR -> motorEntry = start.entry;
              case BATTERY_VOLTAGE -> batteryEntry = start.entry;
              case ENABLED -> enabledEntry = start.entry;
              case FIRE_SHOT -> fireEntry = start.entry;
              case DISTANCE_TO_TARGET -> distanceEntry = start.entry;
              default -> {
              }
            }
            switch (start.name) {
              case FEEDER_MOTOR -> ballPathEntries[0] = start.entry;
              case FEEDER_MOTOR_1 -> ballPathEntries[1] = start.entry;
              case SPINDEXER_MOTOR -> ballPathEntries[2] = start.entry;
              case SPINDEXER_MOTOR_1 -> ballPathEntries[3] = start.entry;
              default -> {
              }
            }
            if (start.name.endsWith("DriveCurrentAmps")) {
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("DriveAppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
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
          } else if (entry == distanceEntry) {
            distance = record.getDouble();
          } else if (entry == motorEntry) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
          } else if (entry == fireEntry) {
            boolean nowFiring = record.getBoolean();
            if (nowFiring && !firing && enabled) {
              shots.add(new double[] { distance, goal, speed, battery,
                  driveSupply.values().stream().mapToDouble(Double::doubleValue).sum(),
                  ballPathDraw[0] + ballPathDraw[1], ballPathDraw[2] + ballPathDraw[3] });
            }
            firing = nowFiring;
          } else if (statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          } else if (appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          } else {
            for (int i = 0; i < ballPathEntries.length; i++) {
              if (entry == ballPathEntries[i]) {
                ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
                ballPathDraw[i] = Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET));
              }
            }
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }
    return shots;
  }

  /**
   * Integrates each motor's supply current against bus voltage to say where a
   * match's energy actually goes, and how much of it is spent standing still.
   *
   * <p>
   * The idle column is the interesting one. It is energy drawn while the motor is
   * commanded below a tenth of duty — holding a position, fighting a spring, or
   * simply not switched off — and it buys nothing. Every watt-hour there is a
   * watt-hour of pack the shooter does not get late in a match.
   */
  private static void reviewEnergy(List<File> logs) throws IOException {
    Map<String, double[]> energy = new HashMap<>();
    double totalSeconds = 0;

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> talons = new HashMap<>();
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> lastTime = new HashMap<>();
      int enabledEntry = -1;
      int batteryEntry = -1;
      boolean enabled = false;
      double battery = 12;
      double firstTime = -1;
      double latestTime = -1;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.name.equals(BATTERY_VOLTAGE)) {
              batteryEntry = start.entry;
            } else if (start.type.equals("struct:TalonFXIOData")) {
              talons.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("CurrentAmps")) {
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("AppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          double now = record.getTimestamp() / 1e6;
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
            if (enabled && firstTime < 0) {
              firstTime = now;
            }
          } else if (entry == batteryEntry) {
            battery = record.getDouble();
          } else if (enabled && talons.containsKey(entry)) {
            String motor = talons.get(entry);
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double supply = Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET));
            double duty = Math.abs(buf.getDouble(DUTY_CYCLE_OFFSET));
            accumulate(energy, motor, supply * battery, duty < 0.1, now, lastTime);
            latestTime = now;
          } else if (enabled && statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            Double applied = volts.get(base);
            if (applied != null && battery > 4) {
              accumulate(energy, base + "(est)", stator.get(base) * applied, applied < 1.0, now,
                  lastTime);
            }
          } else if (enabled && appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
      if (firstTime > 0 && latestTime > firstTime) {
        totalSeconds += latestTime - firstTime;
      }
    }

    if (energy.isEmpty()) {
      System.out.println("no motor data");
      return;
    }
    double grand = energy.values().stream().mapToDouble(e -> e[0]).sum();
    System.out.printf("%.0f seconds enabled across %d logs, %.1f Wh total through the motors%n",
        totalSeconds, logs.size(), grand / 3600);
    System.out.printf("(a typical 18 Ah battery holds about 216 Wh)%n%n");
    System.out.printf("  %-30s %10s %8s %12s %10s%n",
        "motor", "Wh", "share", "idle Wh", "idle share");
    energy.entrySet().stream()
        .sorted((a, b) -> Double.compare(b.getValue()[0], a.getValue()[0]))
        .forEach(e -> System.out.printf("  %-30s %9.1f %7.0f%% %11.1f %9.0f%%%n",
            e.getKey(), e.getValue()[0] / 3600, 100 * e.getValue()[0] / grand,
            e.getValue()[1] / 3600, 100 * e.getValue()[1] / Math.max(1e-9, e.getValue()[0])));
  }

  private static void accumulate(Map<String, double[]> energy, String motor, double watts,
      boolean idle, double now, Map<String, Double> lastTime) {
    Double previous = lastTime.put(motor, now);
    if (previous == null) {
      return;
    }
    double step = Math.min(0.1, now - previous);
    if (step <= 0) {
      return;
    }
    double[] totals = energy.computeIfAbsent(motor, k -> new double[2]);
    totals[0] += watts * step;
    if (idle) {
      totals[1] += watts * step;
    }
  }

  /**
   * Finds the fastest the flywheel was ever observed to turn at each bus
   * voltage, and how hard it was being pushed to get there.
   *
   * <p>
   * A Kraken X60 free-spins at 100 rps and the flywheel is direct driven, so the
   * electrical ceiling is far above anything the shot map asks for. What matters
   * is the speed at which the motor's torque is balanced by the wheel's own drag,
   * which is a real ceiling with an entirely different fix. This separates the
   * two: if the fastest observed speed rises with voltage the wheel is
   * voltage-bound, and if it flattens off the wheel is drag-bound.
   */
  private static void reviewCeiling(List<File> logs) throws IOException {
    // Buckets of 0.5V from 7.0V.
    int buckets = 12;
    double[] fastest = new double[buckets];
    double[] fastestGoal = new double[buckets];
    double[] currentAtFastest = new double[buckets];
    long[] samples = new long[buckets];
    double highestEver = 0;
    double voltageAtHighest = 0;

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      int goalEntry = -1;
      int motorEntry = -1;
      int batteryEntry = -1;
      int enabledEntry = -1;
      boolean enabled = false;
      double goal = 0;
      double battery = 12;

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            switch (start.name) {
              case FLYWHEEL_GOAL -> goalEntry = start.entry;
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
          } else if (enabled && entry == motorEntry && battery > 7.0) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            double speed = Math.abs(buf.getDouble(VELOCITY_OFFSET));
            double stator = Math.abs(buf.getDouble(STATOR_CURRENT_OFFSET));
            int bucket = Math.min(buckets - 1, (int) ((battery - 7.0) * 2));
            samples[bucket]++;
            if (speed > fastest[bucket]) {
              fastest[bucket] = speed;
              fastestGoal[bucket] = goal;
              currentAtFastest[bucket] = stator;
            }
            if (speed > highestEver) {
              highestEver = speed;
              voltageAtHighest = battery;
            }
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    System.out.printf("fastest the flywheel ever turned: %.1f rps, at %.2fV%n", highestEver,
        voltageAtHighest);
    System.out.printf("a Kraken X60 free-spins at 100 rps, and this is direct driven%n%n");
    System.out.printf("  %-14s %10s %12s %14s %14s%n",
        "battery", "samples", "fastest", "goal then", "stator then");
    for (int i = 0; i < buckets; i++) {
      if (samples[i] < 100) {
        continue;
      }
      System.out.printf("  %4.1f-%4.1fV %10d %9.1f rps %11.1f rps %11.1f A%n",
          7.0 + i * 0.5, 7.5 + i * 0.5, samples[i], fastest[i], fastestGoal[i],
          currentAtFastest[i]);
    }
  }

  /**
   * Looks at every moment the bus fell far enough to matter, and asks what was
   * drawing at the time and whether capping it would have kept the robot up.
   *
   * <p>
   * This is the failure the drive team actually reports — browning out in the
   * last thirty seconds and being unable to shoot — and it is not the same thing
   * as a flywheel that recovers slowly. A shot fired slightly slow lands short; a
   * brownout stops the robot doing anything at all. The accuracy model scores the
   * first and is blind to the second.
   */
  private static void reviewBrownout(List<File> logs) throws IOException {
    // Sag events, one per excursion below the threshold rather than per sample.
    List<double[]> events = new ArrayList<>();
    Map<String, Double> mechanismTotals = new HashMap<>();
    final double threshold = 6.5;

    for (File log : logs) {
      DataLogReader reader = new DataLogReader(log.getAbsolutePath());
      Map<Integer, String> statorAmps = new HashMap<>();
      Map<Integer, String> appliedVolts = new HashMap<>();
      Map<String, Double> stator = new HashMap<>();
      Map<String, Double> volts = new HashMap<>();
      Map<String, Double> driveSupply = new HashMap<>();
      Map<Integer, String> talons = new HashMap<>();
      Map<String, Double> mechanismSupply = new HashMap<>();
      int voltageEntry = -1;
      int enabledEntry = -1;
      int matchTimeEntry = -1;
      boolean enabled = false;
      boolean sagging = false;
      double matchTime = Double.NaN;
      double battery = 12;
      double worstThisEvent = 12;
      double driveAtWorst = 0;
      Map<String, Double> mechanismsAtWorst = new HashMap<>();

      try {
        for (DataLogRecord record : reader) {
          if (record.isStart()) {
            var start = record.getStartData();
            if (start.name.equals(BATTERY_VOLTAGE)) {
              voltageEntry = start.entry;
            } else if (start.name.equals(ENABLED)) {
              enabledEntry = start.entry;
            } else if (start.name.equals(MATCH_TIME)) {
              matchTimeEntry = start.entry;
            } else if (start.name.endsWith("CurrentAmps")) {
              // Both drive and steer. Steer was missing from this accounting at
              // first, which understated the drivetrain's share of a brownout.
              statorAmps.put(start.entry, shortName(start.name));
            } else if (start.name.endsWith("AppliedVolts")) {
              appliedVolts.put(start.entry, shortName(start.name));
            } else if (start.type.equals("struct:TalonFXIOData")) {
              talons.put(start.entry, shortName(start.name).replaceAll("[0-9]*$", "")
                  .replaceAll("MotorData$|Motor$", "").replaceAll("^[A-Za-z]+/", ""));
            }
            continue;
          }
          if (record.isControl()) {
            continue;
          }
          int entry = record.getEntry();
          if (entry == enabledEntry) {
            enabled = record.getBoolean();
          } else if (entry == matchTimeEntry) {
            matchTime = record.getDouble();
          } else if (talons.containsKey(entry)) {
            ByteBuffer buf = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
            mechanismSupply.merge(talons.get(entry) + "@" + entry,
                Math.abs(buf.getDouble(SUPPLY_CURRENT_OFFSET)), (a, b) -> b);
          } else if (entry == voltageEntry) {
            battery = record.getDouble();
            if (!enabled || battery < 4.0) {
              continue;
            }
            double drive = driveSupply.values().stream().mapToDouble(Double::doubleValue).sum();
            if (battery < threshold) {
              if (!sagging || battery < worstThisEvent) {
                worstThisEvent = battery;
                driveAtWorst = drive;
                mechanismsAtWorst.clear();
                for (var e : mechanismSupply.entrySet()) {
                  mechanismsAtWorst.merge(e.getKey().split("@")[0], e.getValue(), Double::sum);
                }
              }
              sagging = true;
            } else if (sagging) {
              events.add(new double[] { worstThisEvent, driveAtWorst, matchTime });
              for (var e : mechanismsAtWorst.entrySet()) {
                mechanismTotals.merge(e.getKey(), e.getValue(), Double::sum);
              }
              sagging = false;
              worstThisEvent = 12;
            }
          } else if (statorAmps.containsKey(entry)) {
            String base = statorAmps.get(entry).replace("CurrentAmps", "");
            stator.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          } else if (appliedVolts.containsKey(entry)) {
            String base = appliedVolts.get(entry).replace("AppliedVolts", "");
            volts.put(base, Math.abs(record.getDouble()));
            updateDriveSupply(base, stator, volts, battery, driveSupply);
          }
        }
      } catch (RuntimeException e) {
        // truncated log; keep what was read
      }
    }

    if (events.isEmpty()) {
      System.out.println("no sag events below " + threshold + "V");
      return;
    }
    System.out.printf("%d excursions below %.1fV while enabled%n%n", events.size(), threshold);

    long late = events.stream().filter(e -> !Double.isNaN(e[2]) && e[2] < 30).count();
    long withMatchTime = events.stream().filter(e -> !Double.isNaN(e[2])).count();
    System.out.printf("  %d of %d with a match clock happened in the last 30 seconds (%.0f%%)%n",
        late, withMatchTime, 100.0 * late / Math.max(1, withMatchTime));

    List<Double> drives = new ArrayList<>(events.stream().map(e -> e[1]).sorted().toList());
    System.out.printf("  drivetrain draw at the worst point: median %.0fA, 75th %.0fA, peak %.0fA%n%n",
        drives.get(drives.size() / 2), drives.get(drives.size() * 3 / 4),
        drives.get(drives.size() - 1));

    System.out.printf("  what else was drawing at the worst point, averaged over the %d events:%n",
        events.size());
    double drivetrainMean = drives.stream().mapToDouble(Double::doubleValue).sum() / events.size();
    System.out.printf("    %-22s %8.0fA%n", "drivetrain, drive+steer", drivetrainMean);
    mechanismTotals.entrySet().stream()
        .sorted((a, b) -> Double.compare(b.getValue(), a.getValue()))
        .forEach(e -> System.out.printf("    %-22s %8.0fA%n", e.getKey(),
            e.getValue() / events.size()));
    double mechanismMean = mechanismTotals.values().stream().mapToDouble(Double::doubleValue).sum()
        / events.size();
    System.out.printf("    %-22s %8.0fA  (%.0f%% of it is drivetrain)%n%n", "everything above",
        drivetrainMean + mechanismMean, 100 * drivetrainMean / (drivetrainMean + mechanismMean));

    System.out.printf("  %-10s %14s %16s%n", "drive cap", "still below", "prevented");
    for (double cap : new double[] { 45, 20, 10 }) {
      long stillBelow = events.stream()
          .filter(e -> e[0] + Math.max(0, e[1] - 4 * cap) * BATTERY_RESISTANCE < threshold)
          .count();
      System.out.printf("  %8.0fA %13d %15d%n", cap, stillBelow, events.size() - stillBelow);
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
