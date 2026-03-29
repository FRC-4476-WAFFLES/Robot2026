// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoSegmentChooser {
  private final LoggedDashboardChooser<Optional<AutoSegment>> chooser;
  private int options = 0;

  public AutoSegmentChooser(String name) {
    chooser = new LoggedDashboardChooser<>(name);
    chooser.addDefaultOption("None", Optional.empty());
  }

  public AutoSegmentChooser addOption(String key, AutoSegment segment) {
    if (options == 0) {
      chooser.addDefaultOption(key, Optional.of(segment)); // Overwrite the first default once actually added
    } else {
      chooser.addOption(key, Optional.of(segment));
    }
    options++;
    return this;
  }

  public AutoSegmentChooser onChange(Runnable callback) {
    chooser.onChange(var -> callback.run());
    return this;
  }

  public Optional<AutoSegment> get() {
    var currentSegments = chooser.get();
    if (currentSegments != null) {
      return currentSegments;
    }
    return Optional.empty();
  }

  public Optional<List<BlueRelativeTarget>> getTargets() {
    var segmentOptional = get();
    if (segmentOptional.isPresent()) {
      return Optional.of(segmentOptional.get().getTargets());
    }
    return Optional.empty();
  }
}
