// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ExpanderConstants;
import frc.robot.data.Constants.ExpanderConstants.ExpanderPosition;
import frc.robot.utils.lib.EpochTimer;

public class Intake extends SubsystemBase {
  public static enum ExpanderState {
    STOWED,
    EXTENDED,
    INTAKING,
    AGITATING,
    FULLY_AGITATING,
    SMART_AGITATING,
    OUTTAKE_AGITATING
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Expander Zeroed")
  private boolean expanderZeroed = true; // Assume started against hard stop
  @AutoLogOutput(key = "Intake/Intake Goal Velocity")
  private double intakeGoalDutyCycle = 0;
  @AutoLogOutput(key = "Intake/Expander Setpoint")
  private double expanderSetpoint = 0;

  private boolean zeroingTriggered = false;

  public Intake(IntakeIO io) {
    this.io = io;

    new Trigger(() -> inputs.expanderMotor.torqueCurrent() > ExpanderConstants.ZERO_TORQUE_CURRENT)
        .debounce(ExpanderConstants.ZERO_DEBOUNCE)
        .onTrue(Commands.runOnce(() -> zeroingTriggered = true));
  }

  @Override
  public void periodic() {
    EpochTimer.BeginEpoch("Intake");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Intake", inputs);

      if (!RobotContainer.state.robotEnabled()) {
        io.runIntakeDutyCycle(0);
        return;
      }

      io.runIntakeDutyCycle(intakeGoalDutyCycle);

      if (!expanderZeroed) {
        // Zeroing: run expander into hard stop until torque threshold is held
        io.runExpanderDutyCycle(ExpanderConstants.ZERO_DUTY_CYCLE);
        if (zeroingTriggered) {
          io.setExpanderPosition(ExpanderConstants.ZERO_POSITION);
          expanderZeroed = true;
          zeroingTriggered = false;
        }
      } else {
        io.runExpanderPosition(expanderSetpoint / 360);
      }
    }
    EpochTimer.EndEpoch("Intake");
  }

  // Public API
  public void zeroExpander() { // Only for manual zeroing
    expanderZeroed = false;
  }

  public void setIntakeDutyCycle(double dutyCycle) {
    intakeGoalDutyCycle = dutyCycle;
  }

  public void setExpanderSetpoint(double setpoint) {
    expanderSetpoint = setpoint;
  }

  public void setExpanderSetpoint(ExpanderPosition setpoint) {
    expanderSetpoint = setpoint.getDegrees();
  }

  public double getExpanderPosition() {
    return inputs.expanderMotor.position();
  }

  public double getExpanderVelocity() {
    return inputs.expanderMotor.velocity();
  }

  public Command extend() {
    return Commands.runOnce(() -> RobotContainer.state.setExpanderState(ExpanderState.EXTENDED));
  }

  public Command stow() {
    return Commands.runOnce(() -> RobotContainer.state.setExpanderState(ExpanderState.STOWED));
  }

  public Command agitate() {
    return Commands.runOnce(() -> RobotContainer.state.setExpanderState(ExpanderState.AGITATING));
  }

  public Command toggleExtended() {
    return Commands.runOnce(() -> {
      if (RobotContainer.state.getExpanderState() == ExpanderState.STOWED) {
        RobotContainer.state.setExpanderState(ExpanderState.EXTENDED);
      } else {
        RobotContainer.state.setExpanderState(ExpanderState.STOWED);
      }
    }, this);
  }
}
