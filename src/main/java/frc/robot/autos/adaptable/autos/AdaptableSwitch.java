// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoUtils;
import frc.robot.autos.adaptable.AutoSegment;
import frc.robot.autos.adaptable.AutoVisualizer;
import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
import frc.robot.autos.adaptable.choosers.AutoNetworkNumber;
import frc.robot.autos.adaptable.choosers.AutoSegmentChooser;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class AdaptableSwitch {
  private static Command cmd = Commands.none();
  private static String autoClass = "AdaptableSwitchover";
  public static final BlueRelativeTarget start = new BlueRelativeTarget(3.570, 5.8, Rotation2d.fromDegrees(0));

  public static final BlueRelativeTarget crossToNeutral = new BlueRelativeTarget(5.9, 5.8, Rotation2d.fromDegrees(-10))
      .withExitVelocity(3.5);
  public static final BlueRelativeTarget crossToAlliance = new BlueRelativeTarget(5.0, 5.4,
      Rotation2d.fromDegrees(180))
      .withEntryAngle(Rotation2d.fromDegrees(-180))
      .withExitVelocity(1);
  public static final BlueRelativeTarget shooting = new BlueRelativeTarget(3, 5.4, Rotation2d.fromDegrees(180));

  private static final AutoDropdownChooser<Boolean> autoMirroring = new AutoDropdownChooser<Boolean>(
      autoClass + "/Side")
      .addOption("Left", false)
      .addOption("Right", true)
      .onChange(() -> InvalidateCache());

  private static final AutoNetworkNumber autoDelay = new AutoNetworkNumber(autoClass + "/Auto Delay", 0)
      .onChange(() -> InvalidateCache());

  private static final AutoSegmentChooser entryChooser = new AutoSegmentChooser(
      autoClass + "/Entry")
      .addOption("WaitClose", new WaitAttack())
      .onChange(() -> InvalidateCache());

  private static final AutoNetworkNumber preAttackDelay = new AutoNetworkNumber(autoClass + "/Pre Attack Delay", 0)
      .onChange(() -> InvalidateCache());

  private static final AutoSegmentChooser attackChooser = new AutoSegmentChooser(
      autoClass + "/Attack Chooser")
      // .addOption("Attack", null)
      .onChange(() -> InvalidateCache());

  public static void periodic() {
    if (!RobotContainer.state.robotEnabled() && DriverStation.isDSAttached()
        && RobotContainer.autoChooser.getSendableChooser().getSelected() == autoClass) { // Make sure robot doesn't
                                                                                         // get lagged out while
                                                                                         // running
      if (cmd == null) {
        GenerateAuto(false);
      }
    }
  }

  public static void InvalidateCache() {
    cmd = null;
    SmartDashboard.putBoolean(autoClass + "/Cached", false);
  }

  private static void GenerateAuto(boolean immediate) {
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

      // Visualize Path
      AutoVisualizer.VisualizeAuto(start.withMirroring(pathMirrored), allTargets);
    }

    SmartDashboard.putBoolean(autoClass + "/Cached", true);
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

  public static class WaitAttack extends AutoSegment {
    public WaitAttack() {
      add(
          new BlueRelativeTarget(5.8, 4, Rotation2d.fromDegrees(0))
      );
    }
  }
}
