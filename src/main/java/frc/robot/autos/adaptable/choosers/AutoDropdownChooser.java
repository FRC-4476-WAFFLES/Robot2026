// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.choosers;

// Leaf class for fun essentially
public class AutoDropdownChooser<T> extends GenericAutoDropdownChooser<T, AutoDropdownChooser<T>> {
  public AutoDropdownChooser(String name) {
    super(name);
  }
}
