// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers;

// This needs an IO layer.
// I am too lazy to write one.
public class CoralTracking {
    private static final String LIMELIGHT_KEY = VisionConstants.LIMELIGHT_NAME_CORAL;

    private Trigger targetLost = new Trigger(() -> !LimelightHelpers.getTV(LIMELIGHT_KEY))
            .debounce(0.1);

    private double latestTx;
    private double latestTy;
    private boolean hasTarget = false;

    private final NetworkTable softwareTable = NetworkTableInstance.getDefault().getTable("SoftwareInfo")
            .getSubTable("Coral Tracking");
    private final BooleanPublisher hasTargetNT = softwareTable.getBooleanTopic("Has Target").publish();
    private final DoublePublisher tXNT = softwareTable.getDoubleTopic("Tx").publish();
    private final DoublePublisher tYNT = softwareTable.getDoubleTopic("Ty").publish();

    public CoralTracking() {
        latestTx = 0;
        latestTy = 100;
        hasTarget = false;
    }

    public void update() {
        if (LimelightHelpers.getTV(LIMELIGHT_KEY)) {
            if (LimelightHelpers.getTA(LIMELIGHT_KEY) > 0.3) {
                // Reject new targets when our last target was so close it's partly covered by the ground intake
                // [Rejects random flickering to background objects as coral enters intake] 
                if (Math.abs(latestTx) < 15 && Math.abs(latestTy) < 6 && hasTarget) {
                    return;
                }
                // Reject switching to radically different targets
                // [If multiple are in frame, pick just one]
                if (Math.abs(LimelightHelpers.getTX(LIMELIGHT_KEY) - latestTx) > 7 && hasTarget) {
                    return;
                }

                latestTx = LimelightHelpers.getTX(LIMELIGHT_KEY);
                latestTy = LimelightHelpers.getTY(LIMELIGHT_KEY);
                hasTarget = true;
            }
        } else {
            if (targetLost.getAsBoolean()) {
                hasTarget = false;
            }
        }

        tXNT.set(latestTx);
        tYNT.set(latestTy);
        hasTargetNT.set(hasTarget);
    }

    public double getLatestTX() {
        return latestTx;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
}
