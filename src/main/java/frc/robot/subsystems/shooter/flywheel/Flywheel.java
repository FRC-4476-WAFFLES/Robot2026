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
import frc.robot.RobotContainer;
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
  private final Trigger flywheelAtSetpoint = new Trigger(() -> Math
      .abs(inputs.flywheelMotorData0.velocity() - flywheelGoalVelocity) < (FlywheelConstants.RPM_RANGE / 60.0))
      .debounce(0.25);

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
    // return true;
    return flywheelAtSetpoint.getAsBoolean();
  }

  /**
   * Runs on the PowerManager thread, not the main loop — the IO layer's write is
   * a blocking CAN call.
   */
  @Override
  public boolean applyCurrentLimits(double supplyCurrentLimit) {
    return io.setSupplyCurrentLimit(supplyCurrentLimit);
  }

  public Command runSetpointCommand(DoubleSupplier velocity) {
    return Commands.run(() -> {
      runSetpoint(velocity.getAsDouble());
    });
  }
}
