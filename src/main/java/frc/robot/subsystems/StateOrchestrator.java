// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.RobotState.SuperstructureState;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class StateOrchestrator extends VirtualSubsystem {
  // All coordinates are blue alliance relative
  private double shootingLineX = 4.0;
  private double passingLineX = 5.65;

  public StateOrchestrator() {}

  @Override
  public void periodic() {
    determineSuperstructureState();
  }

  private void determineSuperstructureState() {
    var pose = RobotContainer.state.getPose();
    SuperstructureState state = SuperstructureState.DISABLED;
    if (pose.getX() < shootingLineX) {
      state = SuperstructureState.TARGET_HUB;
    }
    if (pose.getX() > passingLineX) {
      state = SuperstructureState.TARGET_PASS;
    }

    RobotContainer.state.setSuperstructureState(state);
  }
}
