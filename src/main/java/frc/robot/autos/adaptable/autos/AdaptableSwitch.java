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

public class AdaptableSwitch extends AdaptableBase {
  public final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));

  public final BlueRelativeTarget crossToNeutral = new BlueRelativeTarget(6.5, 5.8, Rotation2d.fromDegrees(0))
      .withExitVelocity(4.0);
  public final BlueRelativeTarget crossToAlliance = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(1);
  public final BlueRelativeTarget finish = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));
  public final BlueRelativeTarget shootingRotated = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(-140))
      .getMirrored(); // Too lazy to flip coords

  public final BlueRelativeTarget shootingCenter = new BlueRelativeTarget(3, 4, Rotation2d.fromDegrees(-140))
      .withMaxVelocity(0.8)
      .getMirrored(); // Too lazy to flip coords

  private final AutoDropdownChooser<Boolean> autoMirroring = new AutoDropdownChooser<Boolean>(
      autoClass + "/Side")
      .addOption("Left", false)
      .addOption("Right", true)
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber autoDelay = new AutoNetworkNumber(autoClass + "/Auto Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser entryChooser = new AutoSegmentChooser(
      autoClass + "/Entry")
      .addOption("WaitClose", new WaitClose())
      .addOption("WaitMid", new WaitMid())
      .addOption("WaitFar", new WaitFar())
      .onChange(() -> InvalidateCache());

  private final AutoDropdownChooser<Boolean> divertToCenter = new AutoDropdownChooser<Boolean>(autoClass
      + "/Goto Center")
      .addOption("True", true)
      .addOption("False", false)
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber preAttackDelay = new AutoNetworkNumber(autoClass + "/Pre Attack Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser attackChooser = new AutoSegmentChooser(
      autoClass + "/Attack Chooser")
      .addOption("Normal", new NormalAttack())
      .addOption("Deep", new DeepAttack())
      .addOption("Superdeep", new SuperDeepAttack())
      .onChange(() -> InvalidateCache());

  public AdaptableSwitch() {
    super("AdaptableSwitchover");
  }

  @Override
  protected void GenerateAuto(boolean immediate) {
    Boolean pathMirrored = autoMirroring.get();
    if (pathMirrored == null) {
      return;
    }

    ArrayList<BlueRelativeTarget> entryTargets = new ArrayList<>();
    ArrayList<BlueRelativeTarget> attackTargets = new ArrayList<>();

    entryTargets.add(crossToNeutral);
    entryChooser.getTargets().ifPresent(entryTargets::addAll);

    attackChooser.getTargets().ifPresent(attackTargets::addAll);
    attackTargets.add(crossToAlliance.getMirrored()); // Mirrored to go to other bump
    attackTargets.add(finish.getMirrored());

    AutoPath entryPath = new AutoPath(entryTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored);
    AutoPath attackPath = new AutoPath(attackTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored);

    Boolean goToCenter = divertToCenter.get();
    if (goToCenter == null) {
      goToCenter = false;
    }
    Command maybeGoToCenter = goToCenter ? DriveCommands.autoToTarget(shootingCenter.withMirroring(pathMirrored))
        : Commands.none();

    cmd = Commands.sequence(
        AutoUtils.resetOdometry(start.withMirroring(pathMirrored)),
        Commands.waitSeconds(autoDelay.getAsDouble()),

        Commands.deadline(
            Commands.sequence(
                entryPath.follow(),
                Commands.waitSeconds(preAttackDelay.getAsDouble()),
                attackPath.follow(),
                DriveCommands.autoToTarget(shootingRotated.withMirroring(pathMirrored))
            ),
            IntakeCommands.intakeCommand()
        ),

        Commands.parallel(
            ShooterCommands.shootAutoCommand(15),
            maybeGoToCenter
        )
    );

    if (!immediate) {
      ArrayList<BlueRelativeTarget> allTargets = new ArrayList<>();
      allTargets.addAll(entryTargets);
      allTargets.addAll(attackTargets);
      if (goToCenter) {
        allTargets.add(shootingCenter);
      }

      AutoPath visPath = new AutoPath(allTargets.toArray(new BlueRelativeTarget[0])).withMirroring(pathMirrored);

      // Visualize Path
      AutoVisualizer.VisualizeAuto(start.withMirroring(pathMirrored), Arrays.asList(visPath.getTargets()));
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

  public static class WaitClose extends AutoSegment {
    public WaitClose() {
      add(
          new BlueRelativeTarget(6.4, 3.8, Rotation2d.fromDegrees(0))
      );
    }
  }

  public static class WaitMid extends AutoSegment {
    public WaitMid() {
      add(
          new BlueRelativeTarget(6.2, 3, Rotation2d.fromDegrees(0))
      );
    }
  }

  public static class WaitFar extends AutoSegment {
    public WaitFar() {
      add(
          new BlueRelativeTarget(6.2, 2, Rotation2d.fromDegrees(0))
      );
    }
  }

  public static class NormalAttack extends AutoSegment {
    public NormalAttack() {
      add(
          new BlueRelativeTarget(7.9, 3.2, Rotation2d.fromDegrees(-45)),
          new BlueRelativeTarget(7.6, 1.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(1.9)
      );
    }
  }

  public static class DeepAttack extends AutoSegment {
    public DeepAttack() {
      add(
          new BlueRelativeTarget(8.5, 3.2, Rotation2d.fromDegrees(0)),
          new BlueRelativeTarget(8.25, 2.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(0.6),
          new BlueRelativeTarget(8.2, 1.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(1.9)
      );
    }
  }

  public static class SuperDeepAttack extends AutoSegment {
    public SuperDeepAttack() {
      add(
          new BlueRelativeTarget(8.8, 3.2, Rotation2d.fromDegrees(0)),
          new BlueRelativeTarget(8.6, 2.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(0.6),
          new BlueRelativeTarget(8.6, 1.5, Rotation2d.fromDegrees(-90))
              .withMaxVelocity(1.9)
      );
    }
  }
}
