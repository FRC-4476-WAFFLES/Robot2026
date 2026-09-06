package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public TalonFXIOData flywheelMotorData0 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData flywheelMotorData1 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runDutyCycle(double speed) {}

  default void runFlywheelVelocity(double velocity) {}

  default void runFlywheelVelocity(double velocity, double feedForward) {
    runFlywheelVelocity(velocity);
  }

  /**
   * Sets both current limits on both flywheel motors. Blocking CAN write — call
   * it off the main loop.
   *
   * @param statorCurrentLimit amps through the windings, which is what sets the
   *     torque available to accelerate the wheel back to speed
   * @param supplyCurrentLimit amps drawn from the battery
   * @return whether both motors accepted it
   */
  default boolean setCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    return true;
  }
}
