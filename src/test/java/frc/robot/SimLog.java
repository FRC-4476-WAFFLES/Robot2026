// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.TreeSet;

/**
 * Reads a {@code .wpilog} written by a simulation run, so a test or an agent can
 * assert on anything the robot logged rather than only on what a subsystem
 * exposes publicly.
 *
 * <p>
 * Sim runs write to {@code simlogs/} — see {@code Robot}'s SIM branch. Field
 * names are the AdvantageKit keys as they appear in AdvantageScope, for example
 * {@code RealOutputs/Flywheel/At Setpoint} or {@code NT:/AdvantageKit/...}. Call
 * {@link #fields()} to see what a given log actually contains.
 */
public final class SimLog {
  private final Map<String, List<Object>> valuesByField;

  private SimLog(Map<String, List<Object>> valuesByField) {
    this.valuesByField = valuesByField;
  }

  /** The directory sim runs write into. */
  public static Path logDirectory() {
    return Paths.get("simlogs");
  }

  /** Newest {@code .wpilog} in {@code simlogs/}, if there is one. */
  public static Optional<Path> latestLog() {
    Path dir = logDirectory();
    if (!Files.isDirectory(dir)) {
      return Optional.empty();
    }
    try (var stream = Files.list(dir)) {
      return stream
          .filter(p -> p.toString().endsWith(".wpilog"))
          .max(Comparator.comparingLong(p -> p.toFile().lastModified()));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** Opens the newest sim log. */
  public static SimLog openLatest() {
    Path path = latestLog()
        .orElseThrow(() -> new IllegalStateException(
            "No .wpilog in " + logDirectory().toAbsolutePath()
                + " - run a SimHarness test first, and check WPILOGWriter is enabled in Robot's SIM branch"));
    return open(path);
  }

  /** Opens a specific log file. */
  public static SimLog open(Path path) {
    Map<Integer, String> namesByEntry = new HashMap<>();
    Map<Integer, String> typesByEntry = new HashMap<>();
    Map<String, List<Object>> values = new HashMap<>();

    DataLogReader reader;
    try {
      reader = new DataLogReader(path.toString());
    } catch (IOException e) {
      throw new UncheckedIOException("could not read " + path, e);
    }
    if (!reader.isValid()) {
      throw new IllegalStateException(path + " is not a valid WPILOG file");
    }

    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        var start = record.getStartData();
        namesByEntry.put(start.entry, start.name);
        typesByEntry.put(start.entry, start.type);
        continue;
      }
      if (record.isControl()) {
        continue;
      }

      String name = namesByEntry.get(record.getEntry());
      if (name == null) {
        continue;
      }

      Object value = decode(record, typesByEntry.get(record.getEntry()));
      if (value != null) {
        values.computeIfAbsent(name, k -> new ArrayList<>()).add(value);
      }
    }

    return new SimLog(values);
  }

  private static Object decode(DataLogRecord record, String type) {
    if (type == null) {
      return null;
    }
    try {
      switch (type) {
        case "double":
          return record.getDouble();
        case "int64":
          return record.getInteger();
        case "float":
          return (double) record.getFloat();
        case "boolean":
          return record.getBoolean();
        case "string":
        case "json":
          return record.getString();
        default:
          return null; // arrays and structs are not needed yet
      }
    } catch (Exception e) {
      return null; // malformed record, skip rather than fail the whole read
    }
  }

  /** Every field name in the log, sorted. Use this to discover what is available. */
  public TreeSet<String> fields() {
    return new TreeSet<>(valuesByField.keySet());
  }

  /** Field names containing the given text, sorted. */
  public TreeSet<String> fieldsMatching(String substring) {
    TreeSet<String> matches = new TreeSet<>();
    for (String field : valuesByField.keySet()) {
      if (field.contains(substring)) {
        matches.add(field);
      }
    }
    return matches;
  }

  /** Every recorded value for a field, in order. Empty if the field is absent. */
  public List<Object> values(String field) {
    return valuesByField.getOrDefault(field, List.of());
  }

  /** The last recorded numeric value for a field. */
  public OptionalDouble lastDouble(String field) {
    List<Object> recorded = values(field);
    for (int i = recorded.size() - 1; i >= 0; i--) {
      if (recorded.get(i) instanceof Number n) {
        return OptionalDouble.of(n.doubleValue());
      }
    }
    return OptionalDouble.empty();
  }

  /** The largest numeric value a field reached. */
  public OptionalDouble maxDouble(String field) {
    return values(field).stream()
        .filter(Number.class::isInstance)
        .mapToDouble(v -> ((Number) v).doubleValue())
        .max();
  }

  /** Whether a boolean field was ever true. */
  public boolean everTrue(String field) {
    return values(field).stream().anyMatch(v -> Boolean.TRUE.equals(v));
  }

  /** Opens a log in AdvantageScope for a human to look at. Returns false if not installed. */
  public static boolean openInAdvantageScope(Path log) {
    for (String candidate : new String[] {
        System.getProperty("user.home") + "/AppData/Local/Programs/advantagescope/AdvantageScope.exe",
        "C:/Users/Public/wpilib/2026/advantagescope/AdvantageScope (WPILib).exe",
    }) {
      File exe = new File(candidate);
      if (!exe.isFile()) {
        continue;
      }
      try {
        new ProcessBuilder(exe.getAbsolutePath(), log.toAbsolutePath().toString()).start();
        return true;
      } catch (IOException e) {
        return false;
      }
    }
    return false;
  }
}
