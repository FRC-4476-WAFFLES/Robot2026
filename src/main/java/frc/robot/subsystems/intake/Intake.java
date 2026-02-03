// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants.ExpanderConstants;
import frc.robot.data.Constants.ExpanderConstants.ExpanderPosition;

public class Intake extends SubsystemBase {
  public static enum ExpanderState {
    STOWED,
    EXTENDED
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Expander Zeroed")
  private boolean expanderZeroed = true; // Assume started against hard stop
  @AutoLogOutput(key = "Turret/Expander State")
  private ExpanderState expanderState = ExpanderState.STOWED;
  @AutoLogOutput(key = "Turret/Intake Goal Velocity")
  private double intakeGoalVelocity = 0;

  private Trigger zeroingTrigger = new Trigger(
      () -> inputs.expanderMotor.torqueCurrent() > ExpanderConstants.ZERO_TORQUE_CURRENT)
      .debounce(ExpanderConstants.ZERO_DEBOUNCE);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Intake", inputs);

    if (!DriverStation.isEnabled()) {
      io.runIntakeVelocity(0);
      return;
    }

    io.runIntakeVelocity(intakeGoalVelocity);

    if (!expanderZeroed) {
      io.runExpanderDutyCycle(ExpanderConstants.ZERO_DUTY_CYCLE);
      if (zeroingTrigger.getAsBoolean()) {
        io.setExpanderPosition(ExpanderConstants.ZERO_POSITION);
        expanderZeroed = true;
      }
    }

    double expanderSetpoint = ExpanderPosition.STOWED.getDegrees();
    if (expanderState == ExpanderState.EXTENDED) {
      expanderSetpoint = ExpanderPosition.EXTENDED.getDegrees();
    }
    io.runExpanderPosition(expanderSetpoint);
  }

  // Public API
  public void zeroExpander() { // Only for manual zeroing
    expanderZeroed = false;
  }

  public void setIntakeSetpoint(double velocity) {
    intakeGoalVelocity = velocity;
  }

  public ExpanderState getExpanderState() {
    return expanderState;
  }

  public void setExpanderState(ExpanderState state) {
    this.expanderState = state;
  }

  public double getExpanderPosition() {
    return inputs.expanderMotor.position();
  }
}
