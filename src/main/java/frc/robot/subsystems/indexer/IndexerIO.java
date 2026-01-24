package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public TalonFXIOData spindexerMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData feederMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);

  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runDutyCycle(double spindexerSpeed, double feederSpeed) {}

  default void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {}
}