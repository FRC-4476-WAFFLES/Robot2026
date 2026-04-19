// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos.adaptable;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.RobotContainer;
// import frc.robot.autos.adaptable.autos.Adaptable;
// import frc.robot.autos.adaptable.autos.AdaptableSwitch;
// import frc.robot.autos.adaptable.autos.Adaptable.NormalAttack;
// import frc.robot.autos.adaptable.choosers.AutoDropdownChooser;
// import frc.robot.autos.adaptable.choosers.AutoSegmentChooser;

// public class AdaptableManager {
// private static Command cmd = Commands.none();

// private static final AutoSegmentChooser firstAttackDepthChooser = new
// AutoDropdownChooser<AdaptableBase>(
// "AdaptableVariation")
// .addOption("Adaptable", new Adaptable())
// .addOption("Adaptable", new AdaptableSwitch());

// public static void periodic() {
// if (!RobotState.isEnabled() && DriverStation.isDSAttached()
// && RobotContainer.autoChooser.getSendableChooser().getSelected() ==
// "Adaptable") { // Make sure robot doesn't
// // get lagged out while
// // running
// if (cmd == null) {
// GenerateAuto(false);
// }
// }
// }

// public static void InvalidateCache() {
// cmd = null;
// SmartDashboard.putBoolean("AdaptableAuto/Cached", false);
// }

// public static Command run() {

// return Commands.runOnce(() -> {
// if (cmd == null) {
// GenerateAuto(true);
// }

// cmd.onlyWhile(() -> RobotContainer.state.autonomousEnabled()).schedule();
// InvalidateCache();
// });
// }
// }
