// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.utils.vendor.BlueRelativeTarget;

public class AutoSegment {
  protected final ArrayList<BlueRelativeTarget> targets = new ArrayList<>();

  protected final void add(BlueRelativeTarget... targets) {
    this.targets.addAll(Arrays.asList(targets));
  }

  public final ArrayList<BlueRelativeTarget> getTargets() {
    return targets;
  }
}
