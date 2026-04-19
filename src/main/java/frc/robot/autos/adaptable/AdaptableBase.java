// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AdaptableBase {
  protected Command cmd = Commands.none();
  protected String autoClass = "";

  public AdaptableBase(String autoClass) {
    this.autoClass = autoClass;
  }

  public void InvalidateCache() {
    cmd = null;
    SmartDashboard.putBoolean(autoClass + "/Cached", false);
  }

  public void run() {
    if (cmd == null) {
      GenerateAuto(true);
    }

    cmd.onlyWhile(() -> RobotContainer.state.autonomousEnabled()).schedule();
    InvalidateCache();
  }

  protected void GenerateAuto(boolean immediate) {};
}
