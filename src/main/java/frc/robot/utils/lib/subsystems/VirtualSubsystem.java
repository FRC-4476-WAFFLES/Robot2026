// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

/**
 * Provides all periodic methods and cannot be bound to the commandscheduler.
 */
public class VirtualSubsystem implements IExpandedSubsystem {
  private String name;

  public VirtualSubsystem() {
    String name = this.getClass().getSimpleName();
    this.name = name.substring(name.lastIndexOf('.') + 1);
    ExpandedSubsystemManager.RegisterSubsystem(this);
    ExpandedSubsystemManager.RegisterVirtualSubsystem(this); // Wires up the extra periodic method

  }

  public void periodic() {}

  /**
  * Gets the name of this Subsystem.
  *
  * @return Name
  */
  public String getName() {
    return name;
  }
}
