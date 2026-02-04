// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vendor;

import java.util.Optional;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.lib.WafflesUtilities;

public class BlueRelativeTarget {
  protected Pose2d m_reference;
  protected Optional<Rotation2d> m_entryAngle;
  protected double m_velocity;
  protected Optional<Distance> m_rotationRadius;

  private boolean isRedFlipped = false;
  private APTarget target;

  public BlueRelativeTarget(Pose2d target) {
    m_reference = target;
    m_velocity = 0;
    m_entryAngle = Optional.empty();
    m_rotationRadius = Optional.empty();
  }

  public APTarget getFieldRelative() {
    if (isRedFlipped == WafflesUtilities.IsRedAlliance() && target != null) {
      return target;
    }

    target = new APTarget(WafflesUtilities.FlipIfRedAlliance(m_reference));
    if (m_entryAngle.isPresent()) {
      target = target.withEntryAngle(WafflesUtilities.FlipIfRedAlliance(m_entryAngle.get()));
    }
    isRedFlipped = WafflesUtilities.IsRedAlliance();
    return target;
  }

  public BlueRelativeTarget withEntryAngle(Rotation2d entryAngle) {
    m_entryAngle = Optional.of(entryAngle);
    return this;
  }

  public BlueRelativeTarget withRotationRadius(Distance distance) {
    m_rotationRadius = Optional.of(distance);
    return this;
  }

  public BlueRelativeTarget withVelocity(double velocity) {
    m_velocity = velocity;
    return this;
  }

  public BlueRelativeTarget withTarget(Pose2d target) {
    m_reference = target;
    return this;
  }
}
