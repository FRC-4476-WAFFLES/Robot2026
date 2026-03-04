// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.data.TunerConstants;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  public static final int MaxQueueSize = 20;

  // `@Volatile` lets the run loop read `phoenixSignals` at the top of each
  // iteration without taking the signal lock, which avoids blocking registration
  // during the wait/sleep period.
  volatile private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD = TunerConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(MaxQueueSize);

    synchronized (Drive.odometryLock) {
      synchronized (this) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
        System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
        newSignals[phoenixSignals.length] = signal;
        phoenixSignals = newSignals;
        phoenixQueues.add(queue);
      }
    }

    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(MaxQueueSize);
    synchronized (Drive.odometryLock) {
      synchronized (this) {
        genericSignals.add(signal);
        genericQueues.add(queue);
      }
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(MaxQueueSize);

    synchronized (Drive.odometryLock) {
      timestampQueues.add(queue);
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals

      if (isCANFD && phoenixSignals.length > 0) {
        synchronized (this) {
          try {
            BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
          } catch (Exception e) {
            e.printStackTrace();
          }
        }
      } else {
        // "waitForAll" does not support blocking on multiple signals with a bus
        // that is not CAN FD, regardless of Pro licensing. No reasoning for this
        // behavior is provided by the documentation.

        // Sleep without the lock so signal registration is never blocked.
        try {
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
        } catch (Exception e) {
          System.err.println("PhoenixOdometryThread waitForAll error");
          e.printStackTrace();
        }

        synchronized (this) {
          try {
            if (phoenixSignals.length > 0)
              BaseStatusSignal.refreshAll(phoenixSignals);
          } catch (Exception e) {
            System.err.println("PhoenixOdometryThread refreshAll error");
            e.printStackTrace();
          }
        }
      }

      // Save new data to queues
      synchronized (Drive.odometryLock) {
        // Pinning reference once to avoid repeated volatile acesses that invoke
        // overhead
        var signals = phoenixSignals;

        // Sample timestamp is current FPGA time minus average CAN latency
        // Default timestamps from Phoenix are NOT compatible with
        // FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.length > 0) {
          timestamp -= totalLatency / signals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < signals.length; i++) {
          phoenixQueues.get(i).offer(signals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    }
  }
}
