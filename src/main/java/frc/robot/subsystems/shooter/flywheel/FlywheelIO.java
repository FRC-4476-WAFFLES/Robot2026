package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public TalonFXIOData flywheelMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runDutyCycle(double speed) {}

  default void runFlywheelVelocity(double velocity) {}
}
