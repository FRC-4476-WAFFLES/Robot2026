// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.choosers;

import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class GenericAutoDropdownChooser<T, SELF extends GenericAutoDropdownChooser<T, SELF>> { // Really cool type
                                                                                               // shenanigans for method
                                                                                               // chaining in subclass
  protected final LoggedDashboardChooser<T> chooser;
  protected int options = 0;

  public GenericAutoDropdownChooser(String name) {
    chooser = new LoggedDashboardChooser<>(name);
  }

  @SuppressWarnings("unchecked")
  public SELF addOption(String key, T value) {
    if (options == 0) {
      chooser.addDefaultOption(key, value);
    } else {
      chooser.addOption(key, value);
    }
    options++;
    return (SELF) this;
  }

  @SuppressWarnings("unchecked")
  public SELF onChange(Runnable callback) {
    chooser.onChange(var -> callback.run());
    return (SELF) this;
  }

  @SuppressWarnings("unchecked")
  public SELF onChange(Consumer<T> callback) {
    chooser.onChange(var -> callback.accept(chooser.get()));
    return (SELF) this;
  }

  public T get() {
    var currentSegments = chooser.get();
    return currentSegments;
  }
}
