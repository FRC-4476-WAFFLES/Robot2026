// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.data.Constants.PhysicalConstants;

public class TurretIOSim extends TurretIOTalonFX {
  /*
   * The real turret is driven by MotionMagic, which holds it to
   * TurretConstants.MAX_VELOCITY and MAX_ACCELERATION. The simulation used to
   * ignore both, running its own proportional controller against an inertia of
   * 0.001 kg m^2 -- so the simulated turret snapped to any setpoint in a couple
   * of loops and behaved nothing like the real one. Reproducing the profile
   * instead is both simpler and closer.
   */
  private double position = 0;
  private double velocity = 0;
  private double goalPosition = 0;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double dt = CodeConstants.PERIODIC_LOOP_TIME;
    double maxVelocity = TurretConstants.MAX_VELOCITY;
    double maxAcceleration = TurretConstants.MAX_ACCELERATION;

    // Trapezoidal profile: head for the goal at full acceleration, but never
    // faster than can be stopped in the distance that remains.
    double error = goalPosition - position;
    double stoppingSpeed = Math.sqrt(2 * maxAcceleration * Math.abs(error));
    double targetVelocity = Math.signum(error) * Math.min(maxVelocity, stoppingSpeed);

    double velocityChange = MathUtil.clamp(targetVelocity - velocity,
        -maxAcceleration * dt, maxAcceleration * dt);
    velocity += velocityChange;
    position += velocity * dt;

    // Settle rather than dithering around the goal.
    if (Math.abs(error) < 1e-4 && Math.abs(velocity) < 1e-3) {
      position = goalPosition;
      velocity = 0;
    }

    var talonFXSim = turret.getSimState();
    Logger.recordOutput("Turret/SimHeading", position);
    talonFXSim.setRawRotorPosition(position * PhysicalConstants.TURRET_REDUCTION);
    talonFXSim.setRotorVelocity(velocity * PhysicalConstants.TURRET_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runSetpoint(double setpoint, double setpointVelocity) {
    goalPosition = setpoint;
    super.runSetpoint(setpoint, setpointVelocity);
  }

  @Override
  public void setPosition(double newPosition) {
    position = newPosition;
    goalPosition = newPosition;
    velocity = 0;
  }
}
