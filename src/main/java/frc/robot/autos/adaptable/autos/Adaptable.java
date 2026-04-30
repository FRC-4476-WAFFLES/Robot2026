// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.autos;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoUtils;
import frc.robot.autos.adaptable.AdaptableBase;
import frc.robot.autos.adaptable.AutoSegment;
import frc.robot.autos.adaptable.AutoVisualizer;
import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
import frc.robot.autos.adaptable.choosers.AutoNetworkNumber;
import frc.robot.autos.adaptable.choosers.AutoSegmentChooser;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class Adaptable extends AdaptableBase {
  public static double PICKUP_VELOCITY = 1.7;

  public static final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));
  public static final BlueRelativeTarget crossToNeutral = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(4);
  public static final BlueRelativeTarget crossToAlliance = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(1);
  public static final BlueRelativeTarget shooting = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));

  public static final BlueRelativeTarget turnShootLeft0 = new BlueRelativeTarget(3, 5.4,
      Rotation2d.fromDegrees(-5));
  // .withMaxRotationRate(3)
  public static final BlueRelativeTarget turnShootLeft1 = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(0));

  public static final BlueRelativeTarget turnShootRight0 = new BlueRelativeTarget(3, 5.4,
      Rotation2d.fromDegrees(5));
  public static final BlueRelativeTarget turnShootRight1 = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(0));

  public static final BlueRelativeTarget shootingCenter = new BlueRelativeTarget(3, 4, Rotation2d.fromDegrees(0))
      .withMaxVelocity(0.8); // Too lazy to flip coords

  // NT
  private final AutoNetworkNumber autoDelay = new AutoNetworkNumber(autoClass + "/Auto Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoDropdownChooser<Boolean> autoMirroring = new AutoDropdownChooser<Boolean>(
      autoClass + "/Side")
      .addOption("Left", false)
      .addOption("Right", true)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser firstAttackDepthChooser = new AutoSegmentChooser(
      autoClass + "/First Attack Depth")
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack())
      .addOption("MidSpatula", new MidSpatula())
      // .addOption("Spatula", new Spatula())
      // .addOption("WeakSpatula", new WeakSpatula())
      // .addOption("Inverted Spatula", new InvertedSpatula())
      .addOption("Wait", new WaitAttack())
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber preSweepDelay = new AutoNetworkNumber(autoClass + "/First Sweep Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser firstSweepChooser = new AutoSegmentChooser(
      autoClass + "/First Sweep")
      .addOption("Normal", new NormalSweep())
      .addOption("Greedy", new GreedySweep())
      .addOption("Rotate Out", new NoSweep())
      .addOption("ShortLoopback", new WeakLoopbackSweep())
      .addOption("MediumLoopback", new MediumLoopbackSweep())
      .addOption("Loopback", new LoopbackSweep())
      .onChange(() -> InvalidateCache());

  private final AutoDropdownChooser<Boolean> divertToCenter = new AutoDropdownChooser<Boolean>(autoClass
      + "/Goto Center")
      .addOption("True", true)
      .addOption("False", false)
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber shootTime = new AutoNetworkNumber(autoClass + "/Shoot Timeout", 9)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser secondAttackDepthChooser = new AutoSegmentChooser(
      autoClass + "/Second Attack Depth")
      .addOption("BLOCKER", new Blocker())
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack())
      // .addOption("SuperDeep", new SuperDeepAttack())
      // .addOption("Spatula", new Spatula())
      .onChange(() -> InvalidateCache());
  private final AutoSegmentChooser secondSweepChooser = new AutoSegmentChooser(
      autoClass + "/Second Sweep")
      .addOption("Normal", new NormalSweep())
      .addOption("Greedy", new GreedySweep())
      .addOption("No Sweep", new NoSweep())
      .addOption("ShortLoopback", new WeakLoopbackSweep())
      .addOption("MediumLoopback", new MediumLoopbackSweep())
      .addOption("Loopback", new LoopbackSweep())
      .onChange(() -> InvalidateCache());

  public Adaptable() {
    super("AdaptableStandard");
  }

  @Override
  protected void GenerateAuto(boolean immediate) {
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
        .withMirroring(pathMirrored);

    boolean blockerAuto = secondAttackDepthChooser.getName() == "BLOCKER";

    if (blockerAuto) {
      firstSweepChooser.getTargets().ifPresent(firstAttackTargets::addAll);
      AutoPath blockPath = new AutoPath(firstAttackTargets.toArray(new BlueRelativeTarget[0]))
          .withMirroring(pathMirrored);

      allFirstPassTargets = firstAttackTargets;

      // Make permanent blocker auto
      collectBalls = Commands.sequence(
          blockPath.follow(),
          DriveCommands.autoToTarget(new Blocker().getTargets().get(0)),
          Commands.parallel(
              DriveCommands.autoToTarget(new Blocker().getTargets().get(1)).repeatedly() // never end
          )
      );

    } else if (preSweepDelay.getAsDouble() > 1e-9) {
      // split up path to add a wait
      AutoPath attackPath = new AutoPath(firstAttackTargets.toArray(new BlueRelativeTarget[0]))
          .withMirroring(pathMirrored);
      AutoPath returnPath = new AutoPath(firstReturnTargets.toArray(new BlueRelativeTarget[0]))
          .withMirroring(pathMirrored);

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

    Command rotateForSecondPass = pathMirrored ? Commands.sequence( // Left side turn
        DriveCommands.autoToTarget(turnShootLeft0.withMirroring(pathMirrored)),
        DriveCommands.autoToTarget(turnShootLeft1.withMirroring(pathMirrored))
    )
        : Commands.sequence( // Right side turn
            DriveCommands.autoToTarget(turnShootRight0.withMirroring(pathMirrored)),
            DriveCommands.autoToTarget(turnShootRight1.withMirroring(pathMirrored))
        );

    Boolean goToCenter = divertToCenter.get();
    if (goToCenter == null) {
      goToCenter = false;
    }
    Command maybeGoToCenter = goToCenter ? DriveCommands.autoToTarget(shootingCenter.withMirroring(pathMirrored))
        : Commands.none();
    Command maybeGoBack = goToCenter ? DriveCommands.autoToTarget(turnShootLeft1.withMirroring(pathMirrored))
        : Commands.none();

    // Auto definition
    cmd = Commands.sequence(
        AutoUtils.resetOdometry(start.withMirroring(pathMirrored)),
        Commands.waitSeconds(autoDelay.getAsDouble()),

        Commands.deadline(
            collectBalls,
            IntakeCommands.intakeCommand()
        ),

        Commands.deadline(
            ShooterCommands.shootAutoCommand(shootTime.getAsDouble() - 2).withTimeout(shootTime.getAsDouble()),
            Commands.sequence(
                rotateForSecondPass,
                maybeGoToCenter
            )
        ),

        Commands.deadline(
            Commands.sequence(
                maybeGoBack,
                secondPass.follow()
            ),
            IntakeCommands.intakeCommand()
        )
    );

    if (!immediate) {
      var fullPath = new ArrayList<>(Arrays.asList(firstPass.getTargets()));
      if (goToCenter) {
        fullPath.add(shootingCenter);
        fullPath.add(turnShootLeft0);
      }
      if (!blockerAuto) {
        fullPath.addAll(Arrays.asList(secondPass.getTargets()));
      }

      // Visualize Path
      AutoVisualizer.VisualizeAuto(start.withMirroring(pathMirrored), fullPath);
    }

    SmartDashboard.putBoolean(autoClass + "/Cached", true);

    // Warn against stupids
    if (RobotContainer.state.getPose().getTranslation()
        .getDistance(start.withMirroring(pathMirrored).getFieldRelativePose().getTranslation()) > 0.3) {
      farFromStart.set(true);
    } else {
      farFromStart.set(false);
    }
  }

  // Attacks
  public static class NormalAttack extends AutoSegment {
    public NormalAttack() {
      add(
          new BlueRelativeTarget(7.6, 6.64, Rotation2d.fromDegrees(-35)),
          new BlueRelativeTarget(7.6, 4.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(PICKUP_VELOCITY)
      );
    }
  }

  public static class DeepAttack extends AutoSegment {
    public DeepAttack() {
      add(
          new BlueRelativeTarget(6.9, 6.4, Rotation2d.fromDegrees(15)),
          new BlueRelativeTarget(8.25, 6.2, Rotation2d.fromDegrees(-90))
              .withEntryAngle(Rotation2d.fromDegrees(-90))
              .withMaxVelocity(2),
          new BlueRelativeTarget(8.2, 4.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(PICKUP_VELOCITY)
      );
    }
  }

  // public static class SuperDeepAttack extends AutoSegment {
  // public SuperDeepAttack() {
  // add(
  // new BlueRelativeTarget(8.6, 6.2, Rotation2d.fromDegrees(0)),
  // new BlueRelativeTarget(8.6, 5.5, Rotation2d.fromDegrees(-90))
  // .withMaxVelocity(0.6),
  // new BlueRelativeTarget(8.6, 4.5, Rotation2d.fromDegrees(-90))
  // .withMaxVelocity(1.9)
  // );
  // }
  // }

  public static class WaitAttack extends AutoSegment {
    public WaitAttack() {
      add(
          new BlueRelativeTarget(6.5, 5.8, Rotation2d.fromDegrees(-90)),
          new BlueRelativeTarget(5.8, 4, Rotation2d.fromDegrees(0))
      );
    }
  }

  public static class Blocker extends AutoSegment {
    public Blocker() {
      add(
          new BlueRelativeTarget(8.35, 4, Rotation2d.fromDegrees(-45))
              .withEntryAngle(Rotation2d.fromDegrees(-45)).withMaxVelocity(2),
          new BlueRelativeTarget(8.35, 4, Rotation2d.fromDegrees(90))
      );
    }
  }

  public static class MidSpatula extends AutoSegment {
    public MidSpatula() {
      add(
          new BlueRelativeTarget(6.9, 3.75, Rotation2d.fromDegrees(0)),
          new BlueRelativeTarget(7.8, 4.5, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(PICKUP_VELOCITY),
          new BlueRelativeTarget(7.8, 6.4, Rotation2d.fromDegrees(90))
              .withMaxVelocity(PICKUP_VELOCITY)
      );
    }
  }

  public static class Spatula extends AutoSegment {
    public Spatula() {
      add(
          new BlueRelativeTarget(7.0, 3.8, Rotation2d.fromDegrees(-20)),
          new BlueRelativeTarget(10.6, 3.5, Rotation2d.fromDegrees(20))
              .withMaxVelocity(3)
              .withEntryAngle(Rotation2d.fromDegrees(20)),
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

  public static class InvertedSpatula extends AutoSegment {
    public InvertedSpatula() {
      add(
          // new BlueRelativeTarget(6.476, 5.640, Rotation2d.fromDegrees(170))
          // .withMaxVelocity(2.5)
          // .withEntryAngle(Rotation2d.fromDegrees(160)),
          new BlueRelativeTarget(9.7, 6.9, Rotation2d.fromDegrees(-10)) // also a rotation releif point
              .withMaxVelocity(2.5)
              .withEntryAngle(Rotation2d.fromDegrees(0)),
          new BlueRelativeTarget(10.3, 6.64, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(-60)),
          new BlueRelativeTarget(10.6, 4.5, Rotation2d.fromDegrees(-80)) // Rotation releif point
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(-90)),
          new BlueRelativeTarget(10.3, 3.5, Rotation2d.fromDegrees(170))
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(-150)),
          new BlueRelativeTarget(7.0, 3.8, Rotation2d.fromDegrees(160))

      );
    }
  }

  public static class WeakSpatula extends AutoSegment {
    public WeakSpatula() {
      add(
          new BlueRelativeTarget(7.0, 4.5, Rotation2d.fromDegrees(0)),
          new BlueRelativeTarget(9.0, 4.5, Rotation2d.fromDegrees(0))
              .withEntryAngle(Rotation2d.fromDegrees(0))
              .withMaxVelocity(2),
          new BlueRelativeTarget(9.7, 3.5, Rotation2d.fromDegrees(-50))
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(-90)),
          new BlueRelativeTarget(10.6, 3.0, Rotation2d.fromDegrees(50))
              .withMaxVelocity(2)
              .withEntryAngle(Rotation2d.fromDegrees(20)),
          new BlueRelativeTarget(10.6, 4.5, Rotation2d.fromDegrees(80)) // Rotation releif point
              .withMaxVelocity(2.5)
              .withEntryAngle(Rotation2d.fromDegrees(90)),
          new BlueRelativeTarget(10.208, 6.64, Rotation2d.fromDegrees(90))
              .withMaxVelocity(2.0)
              .withEntryAngle(Rotation2d.fromDegrees(110)),
          new BlueRelativeTarget(9.6, 6.9, Rotation2d.fromDegrees(-160)) // also a rotation releif point
              .withMaxVelocity(2.0)
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
              .withMaxVelocity(PICKUP_VELOCITY), // Mid swing point
          new BlueRelativeTarget(6.0, 4.5, Rotation2d.fromDegrees(120))
              .withEntryAngle(Rotation2d.fromDegrees(70))
              .withMaxVelocity(1.4)
      );
    }
  }

  public static class LoopbackSweep extends AutoSegment {
    public LoopbackSweep() {
      add(
          new BlueRelativeTarget(9.5, 3.959, Rotation2d.fromDegrees(0))
              .withEntryAngle(Rotation2d.fromDegrees(0))
              .withMaxVelocity(PICKUP_VELOCITY), // Mid swing point
          new BlueRelativeTarget(10.5, 4.9, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(9.365, 5.4, Rotation2d.fromDegrees(180))
              .withEntryAngle(Rotation2d.fromDegrees(-180))
              .withMaxVelocity(1.5)
      );
    }
  }

  public static class MediumLoopbackSweep extends AutoSegment {
    public MediumLoopbackSweep() {
      add(
          new BlueRelativeTarget(8, 3.959, Rotation2d.fromDegrees(0))
              .withEntryAngle(Rotation2d.fromDegrees(0))
              .withMaxVelocity(PICKUP_VELOCITY), // Mid swing point
          new BlueRelativeTarget(9, 4.9, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(7.865, 5.4, Rotation2d.fromDegrees(180))
              .withEntryAngle(Rotation2d.fromDegrees(-180))
              .withMaxVelocity(1.5)
      );
    }
  }

  public static class WeakLoopbackSweep extends AutoSegment {
    public WeakLoopbackSweep() {
      add(
          new BlueRelativeTarget(7.0, 3.959, Rotation2d.fromDegrees(0))
              .withEntryAngle(Rotation2d.fromDegrees(0))
              .withMaxVelocity(PICKUP_VELOCITY), // Mid swing point
          new BlueRelativeTarget(8.0, 4.9, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(90))
              .withMaxVelocity(1.5),
          new BlueRelativeTarget(6.865, 5.4, Rotation2d.fromDegrees(180))
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
              .withMaxVelocity(PICKUP_VELOCITY),
          new BlueRelativeTarget(6.0, 3.7, Rotation2d.fromDegrees(90))
              .withEntryAngle(Rotation2d.fromDegrees(100))
              .withMaxVelocity(1.4)
      );
    }
  }

  public static class NoSweep extends AutoSegment {
    public NoSweep() {
      add(
          new BlueRelativeTarget(7.54, 4.8, Rotation2d.fromDegrees(180))
              .withMaxVelocity(1)
      );
    }
  }
}
