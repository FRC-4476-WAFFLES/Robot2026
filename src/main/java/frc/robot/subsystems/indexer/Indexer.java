// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  @AutoLogOutput(key = "Indexer Goal Velocity")
  private double indexerGoalVelocity = 0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputes/Indexer", inputs);

    if (!DriverStation.isEnabled()) {
      io.runIndexerVelocity(0);
      return;
    }

    io.runIndexerVelocity(indexerGoalVelocity);
  }

  public void setIndexerSetpoint(double velocity) {
    indexerGoalVelocity = velocity;
  }
}
