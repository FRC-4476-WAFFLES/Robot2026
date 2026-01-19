// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.hardware;

import au.grapplerobotics.LaserCan;

public class LaserCANIO {
    private LaserCan laserCAN;

    private DeferredRefresher<Double> laserCanRefresher;

    private static final double UNSET_DEFAULT_DISTANCE = 9999;

    private double distance = UNSET_DEFAULT_DISTANCE;

    public LaserCANIO(String name, int canID) {
        this(name, canID, LaserCan.RangingMode.SHORT, UNSET_DEFAULT_DISTANCE);
    }

    public LaserCANIO(String name, int canID, LaserCan.RangingMode rangingMode, double defaultDistance) {
        distance = defaultDistance;

        // Initialize LaserCan with error handling
        try {
            laserCAN = new LaserCan(canID);
            laserCAN.setRangingMode(rangingMode);
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            // throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
            System.out.println("Failed to initialize LaserCan " + name + ":" + e.getMessage());
            laserCAN = null;
        }

        // Setup deferred refresher
        laserCanRefresher = new DeferredRefresher<Double>(
                name,
                0.02, // 50hz
                () -> {
                    if (laserCAN != null) {
                        var measurement = laserCAN.getMeasurement();
                        if (measurement != null) {
                            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                                return (double) measurement.distance_mm;
                            }
                            if (measurement.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS) {
                                return defaultDistance;
                            }
                            if (measurement.status == LaserCan.LASERCAN_STATUS_NOISE_ISSUE) {
                                return defaultDistance;
                            }
                            if (measurement.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
                                return defaultDistance;
                            }
                            if (measurement.status == LaserCan.LASERCAN_STATUS_WRAPAROUND) {
                                return defaultDistance;
                            }
                        }
                    }
                    return null;
                }
        );
    }

    public double update() {
        var sensorResult = laserCanRefresher.getLatestValue();
        if (sensorResult.isPresent()) {
            distance = sensorResult.get();
        }

        return distance;
    }
}
