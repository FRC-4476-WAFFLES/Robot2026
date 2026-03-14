package frc.robot.subsystems.indexer;

import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.lib.SecondOrderSim;

public class IndexerIOSim extends IndexerIOTalonFX {
  private SecondOrderSim indexerSimState;
  private SecondOrderSim feederSimState;

  private double setpointIndexer = 0;
  private double setpointFeeder = 0;

  public IndexerIOSim() {
    indexerSimState = new SecondOrderSim(2.5, 1, 0, 0);
    feederSimState = new SecondOrderSim(2.5, 1, 0, 0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var indexerSim = indexer1.getSimState();
    var simResult0 = indexerSimState.Evaluate(setpointIndexer, CodeConstants.PERIODIC_LOOP_TIME);
    indexerSim.setRotorVelocity(simResult0.get(0) * PhysicalConstants.INDEXER_REDUCTION);

    var feederSim = feeder.getSimState();
    var simResult1 = feederSimState.Evaluate(setpointFeeder, CodeConstants.PERIODIC_LOOP_TIME);
    feederSim.setRotorVelocity(-simResult1.get(0) * PhysicalConstants.FEEDER_REDUCTION);

    super.updateInputs(inputs);
  }

  @Override
  public void runIndexerVelocity(double spindexerVelocity, double feederVelocity) {
    setpointIndexer = spindexerVelocity;
    setpointFeeder = feederVelocity;
    super.runIndexerVelocity(spindexerVelocity, feederVelocity);
  }
}
