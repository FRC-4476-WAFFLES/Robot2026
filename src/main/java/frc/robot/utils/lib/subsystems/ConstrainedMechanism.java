// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

/**
 * Offers a constrained mechanisms system. Does not offer an early periodic method.
 */
public class ConstrainedMechanism extends ExpandedSubsystem {
  protected double setpoint;
  protected double constrainedSetpoint;

  private final HashMap<String, Boolean> appliedConstraints = new HashMap<String, Boolean>();

  @Override
  public final void earlyPeriodic() {
    constrainedSetpoint = setpoint;
    applyConstraints();
    logAppliedConstraints();
    Logger.recordOutput(getName() + "/Constrained Setpoint", constrainedSetpoint);
    Logger.recordOutput(getName() + "/Setpoint", setpoint);
  }

  /**
   * Applies a setpoint to the mechanism
   * @param value setpoint value
   */
  public void applySetpoint(double value) {
    setpoint = value;
  }

  /**
   * Designed to be overridden, true by default
   * @return Is the mechanism within an allowed deadzone of it's setpoint 
   */
  public boolean atSetpoint() {
    return true;
  }

  /**
   * Gets the current mechanism setpoint
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Should be overridden to apply constraints to the mechanism's setpoint
   * Constraints are applied with runConstraint()
   * eg. runConstraint(exampleConstraintFunc, "exampleConstraint");
   */
  protected void applyConstraints() {
    // eg. runConstraint(() -> 2, "exampleConstraint");
  }

  /**
   * Runs a constraint. Allows tracking what constraints are active for easy debugging. 
   * @param constraint the setpoint after a constraint has been applied
   * @param name the name of the constraint
   */
  protected void runConstraint(Double constraintResult, String name) {
    // A constraint function returns either the setpoint, or some constrained
    // setpoint if needed
    appliedConstraints.put(name, !constraintResult.equals(constrainedSetpoint));
    constrainedSetpoint = constraintResult;
  }

  /* Publishes the currently applied constraints to networktables */
  private void logAppliedConstraints() {
    String output = "";
    for (var constraint : appliedConstraints.entrySet()) {
      if (constraint.getValue()) {
        output += constraint.getKey() + " Active\n";
      }
    }

    if (output.equals("")) {
      output = "No constraints active";
    }
    Logger.recordOutput(getName() + "/Applied Constraints", output);
  }
}
