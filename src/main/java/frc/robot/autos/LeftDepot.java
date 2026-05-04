// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.adaptable.autos.Adaptable;
import frc.robot.commands.drive.AutoPath;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class LeftDepot extends SequentialCommandGroup {
  private final BlueRelativeTarget end = new BlueRelativeTarget(0.60, 5.95, Rotation2d.fromDegrees(180));

  public LeftDepot() {
    ArrayList<BlueRelativeTarget> pathTargets = new ArrayList<>();
    pathTargets.add(Adaptable.crossToNeutral);
    pathTargets.addAll(new Adaptable.NormalAttack().getTargets());
    pathTargets.addAll(new Adaptable.NormalSweep().getTargets());
    pathTargets.add(Adaptable.crossToAlliance);
    pathTargets.add(end);

    AutoPath collectBalls = new AutoPath(pathTargets.toArray(new BlueRelativeTarget[0])).withPreciseFinish();

    addCommands(
        AutoUtils.resetOdometry(Adaptable.start),

        Commands.deadline(
            collectBalls.follow(),
            IntakeCommands.intakeCommand()
        ),

        ShooterCommands.shootAutoCommand(8)
    );
  }
}
