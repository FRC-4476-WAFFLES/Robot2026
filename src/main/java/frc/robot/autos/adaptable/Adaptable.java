// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import java.util.ArrayList;
import java.util.Collections;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoUtils;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class Adaptable {
  private static Command cmd = Commands.none();

  private static final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  private static final BlueRelativeTarget crossToNeutral = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(3.5);
  private static final BlueRelativeTarget crossToAlliance = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(0.7);
  private static final BlueRelativeTarget shooting = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));

  private static final BlueRelativeTarget readyForNextPass = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(0))
      .withMaxRotationRate(2);

  // NT 
  private static final AutoSegmentChooser firstAttackDepthChooser = new AutoSegmentChooser(
      "AdaptableAuto/First Attack Depth")
      .onChange(() -> GenerateAuto())
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack());
  private static final AutoSegmentChooser firstSweepChooser = new AutoSegmentChooser("AdaptableAuto/First Sweep")
      .onChange(() -> GenerateAuto())
      .addOption("Normal", new NormalSweep())
      .addOption("Greedy", new GreedySweep());
  private static final LoggedNetworkNumber preSweepDelay = new LoggedNetworkNumber("AdaptableAuto/First Sweep Delay");

  public static void GenerateAuto() {
    ArrayList<BlueRelativeTarget> allTargets = new ArrayList<>();

    // First attack
    ArrayList<BlueRelativeTarget> firstAttackTargets = new ArrayList<>();
    var firstAttack = firstAttackDepthChooser.get();
    if (firstAttack.isPresent()) {
      firstAttackTargets = firstAttack.get().getTargets();
    }

    // Sweep
    ArrayList<BlueRelativeTarget> sweepTargets = new ArrayList<>();
    var sweep = firstSweepChooser.get();
    if (sweep.isPresent()) {
      sweepTargets = sweep.get().getTargets();
    }

    // Make target list
    allTargets.add(crossToNeutral);
    allTargets.addAll(firstAttackTargets);
    allTargets.addAll(sweepTargets);
    allTargets.add(crossToAlliance);
    allTargets.add(shooting);

    AutoPath collectBalls = new AutoPath(allTargets.toArray(new BlueRelativeTarget[0]))
        .withPreciseFinish();
    AutoPath secondPass = new AutoPath(allTargets.toArray(new BlueRelativeTarget[0]))
        .withPreciseFinish();

    cmd = Commands.sequence(
        AutoUtils.resetOdometry(start),

        Commands.deadline(
            collectBalls.follow(),
            IntakeCommands.intakeCommand()
        ),

        Commands.parallel(
            ShooterCommands.shootAutoCommand(4).withTimeout(9),
            DriveCommands.autoToTarget(readyForNextPass)
        ),

        Commands.deadline(
            secondPass.follow(),
            IntakeCommands.intakeCommand()
        )
    );
  }

  public static Command run() {
    return Commands.defer(() -> {
      if (cmd == null) {
        GenerateAuto();
      }

      Command cmdPin = cmd;
      cmd = null;

      return cmdPin;
    }, Collections.emptySet());
  }

  // Attacks
  public static class NormalAttack extends AutoSegment {
    public NormalAttack() {
      add(
          new BlueRelativeTarget(7.9, 6.2, Rotation2d.fromDegrees(-90)),
          new BlueRelativeTarget(7.6, 4.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(1.7)
      );
    }
  }

  public static class DeepAttack extends AutoSegment {
    public DeepAttack() {
      add(
          new BlueRelativeTarget(8.5, 6.2, Rotation2d.fromDegrees(-90)),
          new BlueRelativeTarget(8.2, 4.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(1.7)
      );
    }
  }

  // Sweeps (Designed to string in after attacks)
  public static class NormalSweep extends AutoSegment {
    public NormalSweep() {
      add(
          new BlueRelativeTarget(6.8, 4.1, Rotation2d.fromDegrees(-180))
              .withEntryAngle(Rotation2d.fromDegrees(-180))
              .withMaxVelocity(1.5), // Mid swing point
          new BlueRelativeTarget(6.0, 4.5, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5)
      );
    }
  }

  public static class GreedySweep extends AutoSegment {
    public GreedySweep() {
      add(
          new BlueRelativeTarget(6.8, 4.0, Rotation2d.fromDegrees(-160))
              .withEntryAngle(Rotation2d.fromDegrees(-160))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(6.0, 3.7, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5)
      );
    }
  }
}
