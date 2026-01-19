// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import java.util.HashMap;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class EpochTimer {
    /* Prepare a networktable to publish telemetry to */
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private static final NetworkTable epochTable = softwareTable.getSubTable("Timing (ms)");

    private static class Epoch {
        public Double lastTime = 0.0; // Seconds
        public DoublePublisher doublePublisher;

        public Epoch(String name) {
            doublePublisher = epochTable.getDoubleTopic(name).publish();
            doublePublisher.set(-1);
        }
    }

    private static HashMap<String, Epoch> EpochMap = new HashMap<>();

    /**
     * Begins a time measurement epoch
     * @param name the name of the epoch
     */
    public static void BeginEpoch(String name) {
        Epoch chosenEpoch;
        if (EpochMap.containsKey(name)) {
            chosenEpoch = EpochMap.get(name);
        } else {
            chosenEpoch = new Epoch(name);
            EpochMap.put(name, chosenEpoch);
        }

        chosenEpoch.lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Ends a time measurement epoch and logs it's length to networktables automatically
     * @param name the name of the epoch
     */
    public static double EndEpoch(String name) {
        if (EpochMap.containsKey(name)) {
            double timeElapsed = Timer.getFPGATimestamp() - EpochMap.get(name).lastTime;
            EpochMap.get(name).doublePublisher.set(timeElapsed * 1000);

            EpochMap.get(name).lastTime = -1.0;
            return timeElapsed;
        } else {
            return -1;
        }
    }
}
