// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Offers an early periodic method as well as a late periodic method in addition to all standard subsystem features.
 */
public class ExpandedSubsystem extends SubsystemBase implements IExpandedSubsystem {
  public ExpandedSubsystem() {
    ExpandedSubsystemManager.RegisterSubsystem(this);
  }
}
