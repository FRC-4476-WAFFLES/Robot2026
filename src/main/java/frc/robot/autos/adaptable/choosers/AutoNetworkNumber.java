// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.choosers;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoNetworkNumber extends LoggedNetworkNumber {
  private double lastValue;
  private Runnable listener = null;

  public AutoNetworkNumber(String path, double defaultValue) {
    super("SmartDashboard/" + path, defaultValue);
  }

  public AutoNetworkNumber onChange(Runnable listener) {
    this.listener = listener;
    return this;
  }

  @Override
  public void periodic() {
    super.periodic();
    var selectedValue = get();
    if (selectedValue != lastValue) {
      lastValue = selectedValue;
      if (listener != null)
        listener.run();
    }
  }
}
