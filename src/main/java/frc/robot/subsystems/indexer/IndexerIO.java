package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public TalonFXIOData indexerMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runDutyCycle(double speed) {}

  default void runIndexerVelocity(double velocity) {}
}