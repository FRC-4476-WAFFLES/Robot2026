// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.adaptable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.adaptable.autos.Adaptable;
import frc.robot.autos.adaptable.autos.AdaptableSwitch;
import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
import frc.robot.utils.vendor.Elastic;

public class AdaptableManager {
  private static final AutoDropdownChooser<AdaptableBase> adaptableVariation = new AutoDropdownChooser<AdaptableBase>(
      "AdaptableVariation")
      .addOption("AdaptableStandard", new Adaptable())
      .addOption("AdaptableSwitchover", new AdaptableSwitch())
      .onChange(() -> SwapTabs());

  public static void periodic() {
    if (!RobotContainer.state.robotEnabled() && DriverStation.isDSAttached()
        && RobotContainer.autoChooser.getSendableChooser().getSelected() == "Adaptable") {
      // Make sure robot doesn't get lagged out while running

      var variation = adaptableVariation.get();
      if (variation != null) {
        if (variation.cmd == null) {
          variation.GenerateAuto(false);
        }
      }
    }
  }

  public static void InvalidateCache() {
    var variation = adaptableVariation.get();
    if (variation != null) {
      variation.InvalidateCache();
    }
  }

  public static void SwapTabs() {
    var variation = adaptableVariation.get();
    if (variation != null) {
      Elastic.selectTab(variation.autoClass);
      variation.InvalidateCache();
    }
  }

  public static Command get() {
    return Commands.runOnce(() -> {
      var variation = adaptableVariation.get();
      if (variation != null) {
        variation.run();
      }
    }
    );
  }
}
