// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vendor;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.lib.WafflesUtilities;
import lombok.Getter;

public class BlueRelativeTarget {
  protected Pose2d m_reference;
  protected Optional<Rotation2d> m_entryAngle;
  protected double m_exitVelocity;
  protected Optional<Distance> m_rotationRadius;

  @Getter
  protected double maxVelocity;

  private boolean isRedFlipped = false;
  private APTarget target;

  public BlueRelativeTarget(double x, double y, Rotation2d rotation) {
    this(new Pose2d(x, y, rotation));
  }

  public BlueRelativeTarget(Pose2d target) {
    m_reference = target;
    m_exitVelocity = 0;
    m_entryAngle = Optional.empty();
    m_rotationRadius = Optional.empty();
    maxVelocity = Double.MAX_VALUE;
  }

  public Pose2d getFieldRelativePose() {
    return getFieldRelative().getReference();
  }

  public APTarget getFieldRelative() {
    if (isRedFlipped == WafflesUtilities.IsRedAlliance() && target != null) {
      return target;
    }

    target = new APTarget(WafflesUtilities.FlipIfRedAlliance(m_reference)).withVelocity(m_exitVelocity);
    if (m_entryAngle.isPresent()) {
      target = target.withEntryAngle(WafflesUtilities.FlipIfRedAlliance(m_entryAngle.get()));
    }
    isRedFlipped = WafflesUtilities.IsRedAlliance();
    return target;
  }

  /**
   * Mirrors pose left/right . Does not flip alliance.
   */
  public BlueRelativeTarget mirror() {
    m_reference = new Pose2d(m_reference.getX(), FlippingUtil.fieldSizeY - m_reference.getY(),
        m_reference.getRotation().unaryMinus());
    if (m_entryAngle.isPresent()) {
      m_entryAngle = Optional.of(m_entryAngle.get().unaryMinus());
    }
    target = null;
    return this;
  }

  public BlueRelativeTarget withEntryAngle(Rotation2d entryAngle) {
    target = null;
    m_entryAngle = Optional.of(entryAngle);
    return this;
  }

  public BlueRelativeTarget withRotationRadius(Distance distance) {
    target = null;

    m_rotationRadius = Optional.of(distance);
    return this;
  }

  public BlueRelativeTarget withExitVelocity(double velocity) {
    target = null;

    m_exitVelocity = velocity;
    return this;
  }

  public BlueRelativeTarget withTarget(Pose2d newTarget) {
    this.target = null;

    m_reference = newTarget;
    return this;
  }

  // Does not invalidate cached APTarget
  public BlueRelativeTarget withMaxVelocity(double maxVelocity) {
    this.maxVelocity = maxVelocity;
    return this;
  }
}
