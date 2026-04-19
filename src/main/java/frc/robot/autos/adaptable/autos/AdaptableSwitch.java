// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.autos;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoUtils;
import frc.robot.autos.adaptable.AdaptableBase;
import frc.robot.autos.adaptable.AutoSegment;
import frc.robot.autos.adaptable.AutoVisualizer;
import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
import frc.robot.autos.adaptable.choosers.AutoNetworkNumber;
import frc.robot.autos.adaptable.choosers.AutoSegmentChooser;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AdaptableSwitch extends AdaptableBase {
  public final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));

  public final BlueRelativeTarget crossToNeutral = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(3.5);
  public final BlueRelativeTarget crossToAlliance = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(1);
  public final BlueRelativeTarget shooting = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));

  private final AutoDropdownChooser<Boolean> autoMirroring = new AutoDropdownChooser<Boolean>(
      autoClass + "/Side")
      .addOption("Left", false)
      .addOption("Right", true)
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber autoDelay = new AutoNetworkNumber(autoClass + "/Auto Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser entryChooser = new AutoSegmentChooser(
      autoClass + "/Entry")
      .addOption("WaitClose", new WaitAttack())
      .onChange(() -> InvalidateCache());

  private final AutoNetworkNumber preAttackDelay = new AutoNetworkNumber(autoClass + "/Pre Attack Delay", 0)
      .onChange(() -> InvalidateCache());

  private final AutoSegmentChooser attackChooser = new AutoSegmentChooser(
      autoClass + "/Attack Chooser")
      // .addOption("Attack", null)
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
    attackTargets.add(shooting.getMirrored());

    AutoPath entryPath = new AutoPath(entryTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored);
    AutoPath attackPath = new AutoPath(attackTargets.toArray(new BlueRelativeTarget[0]))
        .withMirroring(pathMirrored)
        .withPreciseFinish();

    cmd = Commands.sequence(
        AutoUtils.resetOdometry(start.withMirroring(pathMirrored)),
        Commands.waitSeconds(autoDelay.getAsDouble()),

        Commands.deadline(
            Commands.sequence(
                entryPath.follow(),
                Commands.waitSeconds(preAttackDelay.getAsDouble()),
                attackPath.follow()
            ),
            IntakeCommands.intakeCommand()
        ),

        ShooterCommands.shootAutoCommand(15)
    );

    if (!immediate) {
      ArrayList<BlueRelativeTarget> allTargets = new ArrayList<>();
      allTargets.addAll(entryTargets);
      allTargets.addAll(attackTargets);

      AutoPath visPath = new AutoPath(allTargets.toArray(new BlueRelativeTarget[0])).withMirroring(pathMirrored);

      // Visualize Path
      AutoVisualizer.VisualizeAuto(start.withMirroring(pathMirrored), Arrays.asList(visPath.getTargets()));
    }

    SmartDashboard.putBoolean(autoClass + "/Cached", true);
  }

  public static class WaitAttack extends AutoSegment {
    public WaitAttack() {
      add(
          new BlueRelativeTarget(5.8, 4, Rotation2d.fromDegrees(0))
      );
    }
  }
}
