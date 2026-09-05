// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.hardware;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/** Base class for asynchronously refreshing a value which normally blocks while being read */
public class DeferredRefresher<T> {
  private Optional<T> returnValue = Optional.empty();
  private Notifier refreshNotifier;

  /**
   * 
   * @param refreshPeriod Time in seconds between refresh calls
   * @param name The name of the refresher, for printing overrun messages
   * @param checkingFunction Function to update value
   */
  public DeferredRefresher(String name, double refreshPeriod, Supplier<T> checkingFunction) {
    refreshNotifier = new Notifier(() -> {
      double startTime = Timer.getFPGATimestamp();

      var value = checkingFunction.get();
      synchronized (this) {
        returnValue = Optional.ofNullable(value);
      }

      double timeTaken = Timer.getFPGATimestamp() - startTime;
      if (timeTaken > refreshPeriod) {
        // Checking time overrun, notify
        DriverStation.reportWarning("WARNING: [" + name + "] DeferredRefresher period overrun\nTime was: "
            + timeTaken + ", Time should be: " + refreshPeriod, false);
      }
    });

    refreshNotifier.startPeriodic(refreshPeriod);
  }

  /**
   * Retreives the latest value read by the DeferredRefresher
   * @return the latest value
   */
  public synchronized Optional<T> getLatestValue() {
    return returnValue;
  }
}
