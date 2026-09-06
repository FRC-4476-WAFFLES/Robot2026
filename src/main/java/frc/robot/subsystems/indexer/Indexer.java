// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.lib.subsystems.PowerManaged;
import frc.robot.data.Constants.SpindexerConstants.IndexerState;
import frc.robot.utils.lib.EpochTimer;

public class Indexer extends SubsystemBase implements PowerManaged {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  @AutoLogOutput(key = "Indexer/Spindexer Goal Velocity")
  private double spindexerGoalVelocity = 0;

  @AutoLogOutput(key = "Indexer/Feeder Goal Velocity")
  private double feederGoalVelocity = 0;

  /**
   * Applies a supply current limit to the feeder motors only. The spindexer is
   * left alone: it is not in the shot path, and its limits are already the
   * loosest on the robot. Runs on the PowerManager thread, not the main loop.
   */
  @Override
  public boolean applyCurrentLimits(double supplyCurrentLimit) {
    return io.setFeederSupplyCurrentLimit(supplyCurrentLimit);
  }

  /**
   * Applies a supply current limit to the spindexer motors. Separate from the
   * feeder because they are opposites in a budget: the feeder is in the shot
   * path and only ever gets more, the spindexer is upstream of it and is the
   * largest thing that can be capped while a long shot is taken.
   */
  public boolean applySpindexerCurrentLimits(double supplyCurrentLimit) {
    return io.setSpindexerSupplyCurrentLimit(supplyCurrentLimit);
  }

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override

  public void periodic() {
    EpochTimer.BeginEpoch("Indexer");
    {
      io.updateInputs(inputs);
      Logger.processInputs("Inputs/Indexer", inputs);

      if (!RobotContainer.state.robotEnabled()) {
        io.runIndexerVelocity(0, 0);
        return;
      }

      io.runIndexerVelocity(spindexerGoalVelocity, feederGoalVelocity);
    }
    EpochTimer.EndEpoch("Indexer");
  }

  public void setIndexerSetpoint(double spindexerVelocity, double feederVelocity) {
    spindexerGoalVelocity = spindexerVelocity;
    feederGoalVelocity = feederVelocity;
  }

  public Command runIndexerCommand(IndexerState state) {
    return Commands.run(
        () -> runIndexer(state), this);
  }

  public void runIndexer(IndexerState state) {
    setIndexerSetpoint(state.getSpindexerSpeed(), state.getConveyorSpeed());
  }

  public void stopIndexer() {
    setIndexerSetpoint(0, 0);
  }
}
