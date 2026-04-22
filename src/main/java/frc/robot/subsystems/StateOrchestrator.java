// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.RobotState.ShooterState;
import frc.robot.data.FieldConstants;
import frc.robot.utils.lib.WafflesUtilities;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class StateOrchestrator extends VirtualSubsystem {
  // All coordinates are blue alliance relative
  private double shootingLineX = 4.2; // Allow shooting on bump
  private double tagZoneLength = 3.5;
  private double bumpStartLineX = 4.0;
  private double bumpEndLineX = FieldConstants.LinesVertical.neutralZoneNear;
  private double passingLineX = bumpEndLineX + tagZoneLength;

  public StateOrchestrator() {}

  @Override
  public void periodic() {
    determineShooterState();
    determineOnBump();
  }

  private void determineOnBump() {
    var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.state.getPose());
    if (pose.getX() > bumpStartLineX && pose.getX() < bumpEndLineX) {
      RobotContainer.state.onBump = true;
      return;
    }
    if (!RobotContainer.drive.isLevelOnGround()) {
      RobotContainer.state.onBump = true;
      return;
    }
    RobotContainer.state.onBump = false;
  }

  private void determineShooterState() {
    var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.state.getPose());
    ShooterState state = ShooterState.DISABLED;
    if (pose.getX() < shootingLineX) {
      state = ShooterState.TARGET_HUB;
      if (!RobotContainer.drive.isLevelOnGround()) {
        state = ShooterState.TARGET_TAG;
      }
    } else if (pose.getX() > passingLineX) {
      state = ShooterState.TARGET_PASS;
    } else {
      state = ShooterState.TARGET_TAG;

      if (Controls.shootButton.getAsBoolean()) {
        // Switch to passing if shot requested here
        state = ShooterState.TARGET_PASS;
      }
    }

    if (Controls.beachButton.getAsBoolean()) {
      state = ShooterState.HANDLE_BEACHED;
    }

    RobotContainer.state.setShooterState(state);
  }
}
