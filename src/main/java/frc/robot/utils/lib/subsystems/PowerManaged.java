// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

/**
 * A subsystem whose motors can be given a new supply current limit at runtime,
 * so that {@code PowerManager} can hand the battery's capacity to whichever
 * mechanism needs it most right now.
 *
 * <p>
 * The limit is <b>supply</b> current, per motor, in amps. Supply current is what
 * the battery actually delivers; a stator limit does not bound it, because a
 * motor at low duty cycle draws far less from the battery than through its
 * windings.
 *
 * <p>
 * Implementations must be safe to call from a thread other than the main loop —
 * applying a Phoenix configuration is a blocking CAN write and would overrun the
 * loop if done inline.
 */
public interface PowerManaged {
  /**
   * Applies a new supply current limit to every motor this subsystem owns.
   *
   * @param supplyCurrentLimit amps, per motor
   * @return whether every motor accepted the new limit. A caller that ignores
   *     this cannot tell a working budget from a mechanism silently left at the
   *     previous state's limit.
   */
  boolean applyCurrentLimits(double supplyCurrentLimit);
}
