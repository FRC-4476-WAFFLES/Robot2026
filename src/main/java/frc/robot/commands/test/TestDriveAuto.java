// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.test;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.DriveSubsystem;

// public class TestDriveAuto extends SequentialCommandGroup {
//   // Current monitoring for each module
//   private static class ModuleCurrentStats {
//     double minDriveCurrent = Double.MAX_VALUE;
//     double maxDriveCurrent = Double.MIN_VALUE;
//     double totalDriveCurrent = 0;
//     int driveCount = 0;

//     double minSteerCurrent = Double.MAX_VALUE;
//     double maxSteerCurrent = Double.MIN_VALUE;
//     double totalSteerCurrent = 0;
//     int steerCount = 0;
//   }

//   private final ModuleCurrentStats frontLeftStats = new ModuleCurrentStats();
//   private final ModuleCurrentStats frontRightStats = new ModuleCurrentStats();
//   private final ModuleCurrentStats backLeftStats = new ModuleCurrentStats();
//   private final ModuleCurrentStats backRightStats = new ModuleCurrentStats();

//   // NetworkTables publishers
//   private final NetworkTable testTable;
//   private final DoublePublisher[] driveMinCurrentNT = new DoublePublisher[4];
//   private final DoublePublisher[] driveMaxCurrentNT = new DoublePublisher[4];
//   private final DoublePublisher[] driveAvgCurrentNT = new DoublePublisher[4];
//   private final DoublePublisher[] steerMinCurrentNT = new DoublePublisher[4];
//   private final DoublePublisher[] steerMaxCurrentNT = new DoublePublisher[4];
//   private final DoublePublisher[] steerAvgCurrentNT = new DoublePublisher[4];

//   /**
//    * Creates a new TestDriveAuto command.
//    * This command tests various drive movements:
//    * 1. Aligns all wheels forward
//    * 2. Drives forward for 5 seconds
//    * 3. Drives backward for 5 seconds
//    * 4. Rotates clockwise for 5 seconds
//    * 5. Rotates counterclockwise for 5 seconds
//    * 6. Re-aligns wheels forward
//    * 
//    * @param drive The drive subsystem
//    */
//   public TestDriveAuto(DriveSubsystem drive) {
//     addRequirements(drive);

//     // Initialize NetworkTables publishers
//     testTable = NetworkTableInstance.getDefault().getTable("DriveTest");
//     String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

//     for (int i = 0; i < 4; i++) {
//       driveMinCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Drive Min Current (Amps)").publish();
//       driveMaxCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Drive Max Current (Amps)").publish();
//       driveAvgCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Drive Avg Current (Amps)").publish();
//       steerMinCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Steer Min Current (Amps)").publish();
//       steerMaxCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Steer Max Current (Amps)").publish();
//       steerAvgCurrentNT[i] = testTable.getDoubleTopic(moduleNames[i] + " Steer Avg Current (Amps)").publish();
//     }

//     // Create a command to publish final statistics
//     Command publishStats = Commands.runOnce(() -> {
//       ModuleCurrentStats[] stats = {frontLeftStats, frontRightStats, backLeftStats, backRightStats};

//       for (int i = 0; i < 4; i++) {
//         driveMinCurrentNT[i].set(stats[i].minDriveCurrent);
//         driveMaxCurrentNT[i].set(stats[i].maxDriveCurrent);
//         driveAvgCurrentNT[i].set(stats[i].totalDriveCurrent / stats[i].driveCount);
//         steerMinCurrentNT[i].set(stats[i].minSteerCurrent);
//         steerMaxCurrentNT[i].set(stats[i].maxSteerCurrent);
//         steerAvgCurrentNT[i].set(stats[i].totalSteerCurrent / stats[i].steerCount);
//       }
//     });

//     // Create a command to monitor current
//     Runnable monitorCurrents = () -> {
//       var modules = drive.getModules();
//       updateModuleStats(frontLeftStats, modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble(),
//                      modules[0].getSteerMotor().getStatorCurrent().getValueAsDouble());
//       updateModuleStats(frontRightStats, modules[1].getDriveMotor().getStatorCurrent().getValueAsDouble(),
//                      modules[1].getSteerMotor().getStatorCurrent().getValueAsDouble());
//       updateModuleStats(backLeftStats, modules[2].getDriveMotor().getStatorCurrent().getValueAsDouble(),
//                      modules[2].getSteerMotor().getStatorCurrent().getValueAsDouble());
//       updateModuleStats(backRightStats, modules[3].getDriveMotor().getStatorCurrent().getValueAsDouble(),
//                      modules[3].getSteerMotor().getStatorCurrent().getValueAsDouble());
//     };

//     // Add commands to the sequential command group
//     addCommands(
//       Commands.sequence(
//         // Initial alignment (1 second)
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
//           Commands.run(monitorCurrents)
//         ).withTimeout(1.0),

//         // Drive forward (5 seconds)
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(1.0).withVelocityY(0).withRotationalRate(0)),
//           Commands.run(monitorCurrents)
//         ).withTimeout(5.0),

//         // Drive backward (5 seconds)
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(-1.0).withVelocityY(0).withRotationalRate(0)),
//           Commands.run(monitorCurrents)
//         ).withTimeout(5.0),

//         // Steer clockwise test (5 seconds) - continuous steer rotation
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(0)
//             .withVelocityY(0)
//             .withRotationalRate(Math.PI * 2)),  // Rotate at 2π radians/sec (continuous rotation)
//           Commands.run(monitorCurrents)
//         ).withTimeout(5.0),

//         // Steer counter-clockwise test (5 seconds) - continuous steer rotation
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(0)
//             .withVelocityY(0)
//             .withRotationalRate(-Math.PI * 2)),  // Rotate at -2π radians/sec (continuous rotation)
//           Commands.run(monitorCurrents)
//         ).withTimeout(5.0),

//         // Final alignment (1 second)
//         Commands.parallel(
//           drive.applyRequest(() -> new SwerveRequest.FieldCentric()
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
//           Commands.run(monitorCurrents)
//         ).withTimeout(1.0),

//         // Publish final statistics
//         publishStats
//       )
//     );
//   }

//   private void updateModuleStats(ModuleCurrentStats stats, double driveCurrent, double steerCurrent) {
//     // Update drive stats
//     stats.minDriveCurrent = Math.min(stats.minDriveCurrent, driveCurrent);
//     stats.maxDriveCurrent = Math.max(stats.maxDriveCurrent, driveCurrent);
//     stats.totalDriveCurrent += driveCurrent;
//     stats.driveCount++;

//     // Update steer stats
//     stats.minSteerCurrent = Math.min(stats.minSteerCurrent, steerCurrent);
//     stats.maxSteerCurrent = Math.max(stats.maxSteerCurrent, steerCurrent);
//     stats.totalSteerCurrent += steerCurrent;
//     stats.steerCount++;
//   }
// } 