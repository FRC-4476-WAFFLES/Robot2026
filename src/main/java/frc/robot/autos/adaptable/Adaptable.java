// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoUtils;
import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
import frc.robot.autos.adaptable.choosers.AutoNetworkNumber;
import frc.robot.autos.adaptable.choosers.AutoSegmentChooser;
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

  private static final BlueRelativeTarget readyForNextPass0 = new BlueRelativeTarget(3, 5.4,
      Rotation2d.fromDegrees(5))
      .withMaxRotationRate(2);
  private static final BlueRelativeTarget readyForNextPass1 = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(0))
      .withMaxRotationRate(2);

  // NT
  private static final AutoNetworkNumber autoDelay = new AutoNetworkNumber("AdaptableAuto/Auto Delay", 0)
      .onChange(() -> InvalidateCache());

  private static final AutoDropdownChooser<Boolean> autoMirroring = new AutoDropdownChooser<Boolean>(
      "AdaptableAuto/Side")
      .addOption("Left", false)
      .addOption("Right", true)
      .onChange(() -> InvalidateCache());

  private static final AutoSegmentChooser firstAttackDepthChooser = new AutoSegmentChooser(
      "AdaptableAuto/First Attack Depth")
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack())
      .addOption("Spatula", new Spatula())
      .onChange(() -> InvalidateCache());

  private static final AutoNetworkNumber preSweepDelay = new AutoNetworkNumber("AdaptableAuto/First Sweep Delay", 0)
      .onChange(() -> InvalidateCache());

  private static final AutoSegmentChooser firstSweepChooser = new AutoSegmentChooser(
      "AdaptableAuto/First Sweep")
      .addOption("Normal", new NormalSweep())
      .addOption("Greedy", new GreedySweep())
      .addOption("No Sweep", new NoSweep())
      .addOption("Loopback", new LoopbackSweep())
      .onChange(() -> InvalidateCache());

  private static final AutoNetworkNumber shootTime = new AutoNetworkNumber("AdaptableAuto/Shoot Timeout", 9)
      .onChange(() -> InvalidateCache());

  private static final AutoSegmentChooser secondAttackDepthChooser = new AutoSegmentChooser(
      "AdaptableAuto/Second Attack Depth")
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack())
      .addOption("Spatula", new Spatula())
      .onChange(() -> InvalidateCache());
  private static final AutoSegmentChooser secondSweepChooser = new AutoSegmentChooser(
      "AdaptableAuto/Second Sweep")
      .addOption("Normal", new NormalSweep())
      .addOption("Greedy", new GreedySweep())
      .addOption("No Sweep", new NoSweep())
      .addOption("Loopback", new LoopbackSweep())
      .onChange(() -> InvalidateCache());

  public static void periodic() {
    if (!RobotState.isEnabled() && DriverStation.isDSAttached()
        && RobotContainer.autoChooser.getSendableChooser().getSelected() == "Adaptable") { // Make sure robot doesn't
                                                                                           // get lagged out while
                                                                                           // running
      if (cmd == null) {
        GenerateAuto(false);
      }
    }
  }

  public static void InvalidateCache() {
    cmd = null;
    SmartDashboard.putBoolean("AdaptableAuto/Cached", false);
  }

  private static void GenerateAuto(boolean immediate) {
    Boolean pathMirrored = autoMirroring.get();
    if (pathMirrored == null) {
      return;
    }

    // First pass
    ArrayList<BlueRelativeTarget> firstAttackTargets = new ArrayList<>();
    ArrayList<BlueRelativeTarget> firstReturnTargets = new ArrayList<>();

    firstAttackTargets.add(crossToNeutral);
    firstAttackDepthChooser.getTargets().ifPresent(firstAttackTargets::addAll);

    firstSweepChooser.getTargets().ifPresent(firstReturnTargets::addAll);
    firstReturnTargets.add(crossToAlliance);
    firstReturnTargets.add(shooting);

    ArrayList<BlueRelativeTarget> allFirstPassTargets = new ArrayList<>();
    allFirstPassTargets.addAll(firstAttackTargets);
    allFirstPassTargets.addAll(firstReturnTargets);

    Command collectBalls;

    AutoPath firstPass = new AutoPath(allFirstPassTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored)
        .withPreciseFinish();

    if (preSweepDelay.getAsDouble() > 1e-9) {
      // split up path to add a wait
      AutoPath attackPath = new AutoPath(firstAttackTargets.toArray(new BlueRelativeTarget[0]))
          .withMirroring(pathMirrored);
      AutoPath returnPath = new AutoPath(firstReturnTargets.toArray(new BlueRelativeTarget[0]))
          .withMirroring(pathMirrored)
          .withPreciseFinish();

      collectBalls = Commands.sequence(
          attackPath.follow(),
          Commands.waitSeconds(preSweepDelay.getAsDouble()),
          returnPath.follow()
      );
    } else {
      collectBalls = firstPass.follow();
    }

    // Second pass
    ArrayList<BlueRelativeTarget> allSecondPassTargets = new ArrayList<>();

    allSecondPassTargets.add(crossToNeutral);
    secondAttackDepthChooser.getTargets().ifPresent(allSecondPassTargets::addAll);
    secondSweepChooser.getTargets().ifPresent(allSecondPassTargets::addAll);
    allSecondPassTargets.add(crossToAlliance);
    allSecondPassTargets.add(shooting);

    AutoPath secondPass = new AutoPath(allSecondPassTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored)
        .withPreciseFinish();

    // Auto definition
    cmd = Commands.sequence(
        AutoUtils.resetOdometry(start.withMirroring(pathMirrored)),
        Commands.waitSeconds(autoDelay.getAsDouble()),

        Commands.deadline(
            collectBalls,
            IntakeCommands.intakeCommand()
        ),

        Commands.deadline(
            ShooterCommands.shootAutoCommand(4).withTimeout(shootTime.getAsDouble()),
            Commands.sequence(
                DriveCommands.autoToTarget(readyForNextPass0.withMirroring(pathMirrored)),
                DriveCommands.autoToTarget(readyForNextPass1.withMirroring(pathMirrored))
            )
        ),

        Commands.deadline(
            secondPass.follow(),
            IntakeCommands.intakeCommand()
        )
    );

    if (!immediate) {
      // Visualize Path
      AutoVisualizer.VisualizeAuto(start.withMirroring(pathMirrored), Arrays.asList(firstPass.getTargets()));
    }

    SmartDashboard.putBoolean("AdaptableAuto/Cached", true);
  }

  public static Command run() {
    return Commands.runOnce(() -> {
      if (cmd == null) {
        GenerateAuto(true);
      }

      cmd.onlyWhile(() -> RobotContainer.state.autonomousEnabled()).schedule();
      InvalidateCache();
    });
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

  public static class Spatula extends AutoSegment {
    public Spatula() {
      add(
          new BlueRelativeTarget(7.0, 4.45, Rotation2d.fromDegrees(-20)),
          new BlueRelativeTarget(10.6, 3.5, Rotation2d.fromDegrees(-20))
              .withMaxVelocity(3)
              .withEntryAngle(Rotation2d.fromDegrees(-20)),
          new BlueRelativeTarget(10.6, 4.5, Rotation2d.fromDegrees(80)) // Rotation releif point
              .withMaxVelocity(1.0)
              .withEntryAngle(Rotation2d.fromDegrees(90)),
          new BlueRelativeTarget(10.208, 6.64, Rotation2d.fromDegrees(90))
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(110)),
          new BlueRelativeTarget(9.6, 6.9, Rotation2d.fromDegrees(-160)) // also a rotation releif point
              .withMaxVelocity(1.2)
              .withEntryAngle(Rotation2d.fromDegrees(180)),
          new BlueRelativeTarget(6.476, 5.640, Rotation2d.fromDegrees(-170))
              .withMaxVelocity(2.5)
              .withEntryAngle(Rotation2d.fromDegrees(-160))
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

  public static class LoopbackSweep extends AutoSegment {
    public LoopbackSweep() {
      add(
          new BlueRelativeTarget(9.5, 3.959, Rotation2d.fromDegrees(0))
              .withEntryAngle(Rotation2d.fromDegrees(0))
              .withMaxVelocity(1.5), // Mid swing point
          new BlueRelativeTarget(10.5, 4.9, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(9.365, 5.4, Rotation2d.fromDegrees(180))
              .withEntryAngle(Rotation2d.fromDegrees(-180))
              .withMaxVelocity(1.5)
      );
    }
  }

  public static class GreedySweep extends AutoSegment {
    public GreedySweep() {
      add(
          new BlueRelativeTarget(7.2, 3.7, Rotation2d.fromDegrees(-160))
              .withEntryAngle(Rotation2d.fromDegrees(-140))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(6.0, 3.7, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(100))
              .withMaxVelocity(1.5)
      );
    }
  }

  public static class NoSweep extends AutoSegment {
    public NoSweep() {}
  }
}
