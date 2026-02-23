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

public class BlueRelativeTarget {
  protected Pose2d m_reference;
  protected Optional<Rotation2d> m_entryAngle;
  protected double m_velocity;
  protected Optional<Distance> m_rotationRadius;

  private boolean isRedFlipped = false;
  private APTarget target;

  public BlueRelativeTarget(double x, double y, Rotation2d rotation) {
    this(new Pose2d(x, y, rotation));
  }

  public BlueRelativeTarget(Pose2d target) {
    m_reference = target;
    m_velocity = 0;
    m_entryAngle = Optional.empty();
    m_rotationRadius = Optional.empty();
  }

  public Pose2d getFieldRelativePose() {
    return getFieldRelative().getReference();
  }

  public APTarget getFieldRelative() {
    if (isRedFlipped == WafflesUtilities.IsRedAlliance() && target != null) {
      return target;
    }

    target = new APTarget(WafflesUtilities.FlipIfRedAlliance(m_reference)).withVelocity(m_velocity);
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
        m_reference.getRotation().unaryMinus().plus(Rotation2d.k180deg));
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

  public BlueRelativeTarget withVelocity(double velocity) {
    target = null;

    m_velocity = velocity;
    return this;
  }

  public BlueRelativeTarget withTarget(Pose2d target) {
    this.target = null;

    m_reference = target;
    return this;
  }
}
