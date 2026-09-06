package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public TalonFXIOData indexerMotorData0 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData indexerMotorData1 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData feederMotorData0 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
    public TalonFXIOData feederMotorData1 = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runDutyCycle(double spindexerSpeed) {}

  default void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {}

  /**
   * Sets the supply current limit on both feeder motors. The feeder is left out
   * of any budget cut: a ball entering the shooter at an inconsistent speed makes
   * the shot inconsistent no matter how well the flywheel is holding, so the
   * feeder is only ever given more room, never less. Blocking CAN write — call it
   * off the main loop.
   *
   * @param supplyCurrentLimit amps, per motor
   * @return whether both motors accepted it
   */
  default boolean setFeederSupplyCurrentLimit(double supplyCurrentLimit) {
    return true;
  }

  /**
   * Sets the supply current limit on both spindexer motors. Unlike the feeder,
   * the spindexer is upstream of the shot rather than in it, so it can be capped
   * while a long shot is being taken. Blocking CAN write — call it off the main
   * loop.
   *
   * @param supplyCurrentLimit amps, per motor
   * @return whether both motors accepted it
   */
  default boolean setSpindexerSupplyCurrentLimit(double supplyCurrentLimit) {
    return true;
  }
}