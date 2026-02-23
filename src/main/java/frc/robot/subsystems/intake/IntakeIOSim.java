// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class IntakeIOSim extends IntakeIOTalonFX {
  private final SecondOrderSim expanderSim;
  private final SecondOrderSim intakeSim;

  private double setpointPos;
  private double setpointIntake;

  public IntakeIOSim() {
    expanderSim = new SecondOrderSim(2.5, 1, 0, 0);
    intakeSim = new SecondOrderSim(2.5, 1, 0, 0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var expanderSimState = expander.getSimState();

    var simResultExpander = expanderSim.Evaluate(setpointPos, CodeConstants.PERIODIC_LOOP_TIME);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    expanderSimState.setRawRotorPosition(simResultExpander.get(0) * PhysicalConstants.EXPANDER_REDUCTION);
    expanderSimState.setRotorVelocity(simResultExpander.get(1) * PhysicalConstants.EXPANDER_REDUCTION);

    var intakeSimState = intake0.getSimState();
    var simResultIntake = intakeSim.Evaluate(setpointIntake, CodeConstants.PERIODIC_LOOP_TIME);
    intakeSimState.setRotorVelocity(simResultIntake.get(0) * PhysicalConstants.EXPANDER_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runExpanderPosition(double position) {
    setpointPos = position;
    super.runExpanderPosition(position);
  }

  @Override
  public void runIntakeVelocity(double velocity) {
    setpointIntake = velocity;
    super.runIntakeVelocity(velocity);
  }
}