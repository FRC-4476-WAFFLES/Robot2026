// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotPlanner;
import frc.robot.data.Constants.FlywheelConstants;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.subsystems.PowerManaged;

public class Flywheel extends SubsystemBase implements PowerManaged {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/Flywheel Goal Velocity")
  private double flywheelGoalVelocity = 0;

  private static final double RECOVERY_FF_PER_RPS = 10;
  private static final double RECOVERY_ENTER_THRESHOLD = 3.0;
  private static final double RECOVERY_EXIT_THRESHOLD = 2.0;
  private boolean inRecovery = false;
  /**
   * Ready to put a ball through, for a shot that has to land in the goal.
   *
   * <p>
   * Tight to open and slow to close. The wheel must hold its tolerance for
   * {@code READY_RISING_DEBOUNCE} before this goes true, and must be outside it
   * for {@code READY_FALLING_DEBOUNCE} before it goes false again — so the dip a
   * ball causes on its way out, which happens after the ball has gone, does not
   * shut the gate behind it.
   */
  private final Trigger flywheelAtSetpoint = new Trigger(
      () -> Math.abs(inputs.flywheelMotorData0.velocity() - flywheelGoalVelocity) < velocityTolerance())
      .debounce(FlywheelConstants.READY_RISING_DEBOUNCE, DebounceType.kRising)
      .debounce(FlywheelConstants.READY_FALLING_DEBOUNCE, DebounceType.kFalling);

  /**
   * Ready enough to pass with. Passing aims at a region of floor rather than a
   * goal, so it keeps the old fixed 20 rps window.
   */
  private final Trigger flywheelAtLooseSetpoint = new Trigger(() -> Math
      .abs(inputs.flywheelMotorData0.velocity() - flywheelGoalVelocity) < (FlywheelConstants.RPM_RANGE / 60.0))
      .debounce(FlywheelConstants.READY_RISING_DEBOUNCE);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    EpochTimer.BeginEpoch("Flywheel");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Flywheel", inputs);

      if (!RobotContainer.state.robotEnabled()) {
        io.runFlywheelVelocity(0);
        return;
      }

      double error = flywheelGoalVelocity - inputs.flywheelMotorData0.velocity();
      boolean shooting = RobotContainer.state.shouldFire().getAsBoolean()
          || RobotContainer.state.shouldFireManual().getAsBoolean();

      if (shooting && error > RECOVERY_ENTER_THRESHOLD) {
        inRecovery = true;
      }
      if (error < RECOVERY_EXIT_THRESHOLD || !shooting) {
        inRecovery = false;
      }

      double feedForward = inRecovery ? error * RECOVERY_FF_PER_RPS : 0;

      Logger.recordOutput("Flywheel/Recovery Boost", feedForward);
      io.runFlywheelVelocity(flywheelGoalVelocity, feedForward);
    }
    EpochTimer.EndEpoch("Flywheel");
  }

  public void runSetpoint(double velocity) {
    flywheelGoalVelocity = velocity;
  }

  @AutoLogOutput(key = "Flywheel/At Setpoint")
  public boolean atSetpoint() {
    return flywheelAtSetpoint.getAsBoolean();
  }

  /** The looser check, for passing. */
  @AutoLogOutput(key = "Flywheel/At Loose Setpoint")
  public boolean atLooseSetpoint() {
    return flywheelAtLooseSetpoint.getAsBoolean();
  }

  /**
   * How far off its goal the wheel may be and still land the shot, in rotations
   * per second.
   *
   * <p>
   * Derived rather than fixed, because range error is
   * {@code 2 * distance * speedError / speed}: the same speed error costs far
   * less range up close than it does far out, so one number is wrong at both
   * ends. Validated against event video — at 3.5 m a 4.3 rps deficit landed
   * 0.65 m short and 22 rps landed 3 to 4 m short, both as this predicts.
   */
  @AutoLogOutput(key = "Flywheel/Velocity Tolerance")
  public double velocityTolerance() {
    double distance = ShotPlanner.distanceToTarget();
    if (distance < 1.0 || flywheelGoalVelocity < 1.0) {
      return FlywheelConstants.MAX_VELOCITY_TOLERANCE;
    }
    return MathUtil.clamp(
        flywheelGoalVelocity * FlywheelConstants.ACCEPTABLE_RANGE_ERROR / (2 * distance),
        FlywheelConstants.MIN_VELOCITY_TOLERANCE, FlywheelConstants.MAX_VELOCITY_TOLERANCE);
  }

  /**
   * Runs on the PowerManager thread, not the main loop — the IO layer's write is
   * a blocking CAN call.
   */
  @Override
  public boolean applyCurrentLimits(double supplyCurrentLimit) {
    return applyCurrentLimits(FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, supplyCurrentLimit);
  }

  /**
   * The flywheel needs its stator limit managed as well as its supply limit.
   * Stator current is what accelerates the wheel back to speed after a ball, and
   * measured stator peaks sit right at the configured ceiling, so raising only
   * the supply limit would not give it any more recovery.
   */
  public boolean applyCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    return io.setCurrentLimits(statorCurrentLimit, supplyCurrentLimit);
  }

  public Command runSetpointCommand(DoubleSupplier velocity) {
    return Commands.run(() -> {
      runSetpoint(velocity.getAsDouble());
    });
  }
}
