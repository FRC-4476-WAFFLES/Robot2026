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
import frc.robot.data.Constants.SpindexerConstants.IndexerState;
import frc.robot.utils.lib.EpochTimer;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  @AutoLogOutput(key = "Indexer/Spindexer Goal Velocity")
  private double spindexerGoalVelocity = 0;

  @AutoLogOutput(key = "Indexer/Feeder Goal Velocity")
  private double feederGoalVelocity = 0;

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
