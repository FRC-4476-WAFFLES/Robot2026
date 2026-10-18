// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.RobotState.ShooterState;
import frc.robot.data.Constants.CodeConstants;
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
  /** When the shoot trigger was last held, or -1 if it has not been yet. */
  private double lastShootRequestedAt = -1;

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

    // A robot that does not know where it is cannot aim, so point the turret
    // where a tag should be and let vision find it. Only while the driver is not
    // asking to shoot: they may well be able to see the goal perfectly well, and
    // taking the turret off them mid-cycle is worse than a pose that is a metre
    // out. That holds for a short while after they let go too, since drivers
    // release between shots in a cycle and recovery would otherwise park the
    // flywheel every time. Releases the instant the pose is found or the trigger
    // is pulled. Never in autonomous: no one holds the shoot button there, and
    // the auto shot command would keep feeding into the flywheel this stops.
    if (Controls.shootButton.getAsBoolean()) {
      lastShootRequestedAt = Timer.getTimestamp();
    }
    boolean shotRecentlyRequested = lastShootRequestedAt >= 0
        && Timer.getTimestamp() - lastShootRequestedAt < CodeConstants.POSE_RECOVERY_SHOOT_GRACE_TIME;
    if (RobotContainer.vision.poseLikelyLost() && !shotRecentlyRequested
        && !RobotContainer.state.autonomousEnabled()) {
      state = ShooterState.RECOVER_POSE;
    }

    if (Controls.beachButton.getAsBoolean()) {
      state = ShooterState.HANDLE_BEACHED;
    }

    RobotContainer.state.setShooterState(state);
  }
}
