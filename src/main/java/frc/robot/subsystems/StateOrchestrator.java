// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.RobotState.ShooterState;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class StateOrchestrator extends VirtualSubsystem {
  // All coordinates are blue alliance relative
  private double shootingLineX = 4.0;
  private double passingLineX = 5.65;

  public StateOrchestrator() {}

  @Override
  public void periodic() {
    determineShooterState();
  }

  private void determineShooterState() {
    var pose = RobotContainer.state.getPose();
    ShooterState state = ShooterState.DISABLED;
    if (pose.getX() < shootingLineX) {
      state = ShooterState.TARGET_HUB;
    }
    if (pose.getX() > passingLineX) {
      state = ShooterState.TARGET_PASS;
    }

    RobotContainer.state.setShooterState(state);
  }
}
