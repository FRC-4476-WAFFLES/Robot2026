// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.hardware;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import frc.robot.Robot;

public class PhoenixHelpers {
    // Maps a canbus to an array of status signals on that bus to be refreshed 
    private static HashMap<String, BaseStatusSignal[]> statusSignalBusMap = new HashMap<>();

    public static void RegisterStatusSignals(String Canbus, BaseStatusSignal... baseStatusSignals) {
        if (statusSignalBusMap.containsKey(Canbus)) {
            // Add to existing array
            BaseStatusSignal[] currentSignalArray = statusSignalBusMap.get(Canbus);

            BaseStatusSignal[] newSignalArray = new BaseStatusSignal[currentSignalArray.length
                    + baseStatusSignals.length];
            System.arraycopy(currentSignalArray, 0, newSignalArray, 0, currentSignalArray.length);
            System.arraycopy(baseStatusSignals, 0, newSignalArray, currentSignalArray.length, baseStatusSignals.length);
            statusSignalBusMap.replace(Canbus, newSignalArray);
        } else {
            // Create new array 
            BaseStatusSignal[] newSignalArray = new BaseStatusSignal[baseStatusSignals.length];
            System.arraycopy(baseStatusSignals, 0, newSignalArray, 0, baseStatusSignals.length);

            statusSignalBusMap.put(Canbus, newSignalArray);
        }
    }

    public static void refreshAllSignals() {
        // Loop over each CAN bus and refresh their signals
        for (var entry : statusSignalBusMap.entrySet()) {
            BaseStatusSignal.refreshAll(entry.getValue());
        }
    }

    /** 
     * Attempts the desired task more than once if failed, up to a limit 
     * @param maxAttempts The max number of attempts
     * @param task A supplier for the config function
     */
    public static void tryConfig(int maxAttempts, Supplier<StatusCode> task) {
        for (int i = 0; i < maxAttempts; i++) {
            if (task.get().isOK())
                return;
        }
        Robot.setCANConfigErrorFlag();
    }

    /** 
     * Attempts the desired task more than once if failed, up to 4 times
     * @param task A supplier for the config function
     */
    public static void tryConfig(Supplier<StatusCode> task) {
        tryConfig(4, task);
    }
}
