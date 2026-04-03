// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.choosers;

import java.util.List;
import java.util.Optional;

import frc.robot.autos.adaptable.AutoSegment;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoSegmentChooser extends GenericAutoDropdownChooser<Optional<AutoSegment>, AutoSegmentChooser> {
  public AutoSegmentChooser(String name) {
    super(name);
    chooser.addDefaultOption("None", Optional.empty());
  }

  public AutoSegmentChooser addOption(String key, AutoSegment segment) {
    addOption(key, Optional.of(segment));
    return this;
  }

  @Override
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
