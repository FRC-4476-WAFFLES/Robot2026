package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public TalonFXIOData indexerMotorData1 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData indexerMotorData2 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData feederMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runDutyCycle(double spindexerSpeed) {}

  default void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {}
}