// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.util.DriveFeedforwards;
// import com.pathplanner.lib.util.swerve.SwerveSetpoint;
// import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.data.Constants.PhysicalConstants;
// import frc.robot.subsystems.DriveSubsystem;

// public class DriveTeleop extends Command {
//   private final DoubleSupplier xSupplier;
//   private final DoubleSupplier ySupplier;
//   private final Supplier<Rotation2d> thetaSupplier;

//   private final boolean isSetpointX, isSetpointY, isSetpointTheta;

//   /* PID Controllers used if suppliers are interpreted as setpoints */
//   private PIDController xPidController = new PIDController(3, 0, 0.1);
//   private PIDController yPidController = new PIDController(3, 0, 0.1);
//   private ProfiledPIDController thetaPidController = new ProfiledPIDController(7.0, 0, 0.1, new Constraints(4, 20));

//   private double previousThetaSetpoint;

//   // Unused setpoint generator variables
//   @SuppressWarnings("unused")
//   private final SwerveSetpointGenerator setpointGenerator;
//   @SuppressWarnings("unused")
//   private SwerveSetpoint previousSetpoint;

//   private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
//   private DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

//   /** 
//    * Command that drives the robot field-oriented following velocities given by suppliers 
//    */
//   public DriveTeleop(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier,
//       Supplier<Rotation2d> thetaVelocitySupplier) {
//     this(xVelocitySupplier, false, yVelocitySupplier, false, thetaVelocitySupplier, false);
//   }

//   /** 
//    * Command that drives the robot field-oriented following velocities given by suppliers, with the option of making suppliers setpoints 
//    */
//   public DriveTeleop(DoubleSupplier xSupplier, boolean isSetpointX, DoubleSupplier ySupplier, boolean isSetpointY,
//       Supplier<Rotation2d> thetaSupplier, boolean isSetpointTheta) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driveSubsystem);

//     this.xSupplier = xSupplier;
//     this.ySupplier = ySupplier;
//     this.thetaSupplier = thetaSupplier;

//     this.isSetpointX = isSetpointX;
//     this.isSetpointY = isSetpointY;
//     this.isSetpointTheta = isSetpointTheta;

//     thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

//     setpointGenerator = new SwerveSetpointGenerator(
//         driveSubsystem.PathPlannerConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
//         PhysicalConstants.maxAngularSpeed // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
//     );

//     // Initialize the previous setpoint to the robot's current speeds & module states
//     ChassisSpeeds currentSpeeds = driveSubsystem.getFieldVelocity(); // Method to get current robot-relative chassis speeds

//     var modules = driveSubsystem.getModules();
//     SwerveModuleState[] currentStates = new SwerveModuleState[4];
//     for (int i = 0; i < modules.length; i++) {
//       currentStates[i] = modules[i].getCurrentState();
//     }

//     previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates,
//         DriveFeedforwards.zeros(driveSubsystem.PathPlannerConfig.numModules));
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double speedDeadband = PhysicalConstants.maxSpeed * 0.05;
//     double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.01;

//     var currentPose = driveSubsystem.getPose();

//     // Reset logic for profiled PID controller
//     if (isSetpointTheta) {
//       if (previousThetaSetpoint != thetaSupplier.get().getRadians()) {
//         previousThetaSetpoint = thetaSupplier.get().getRadians();
//         thetaPidController.reset(currentPose.getRotation().getRadians(),
//             RobotContainer.driveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
//       }
//     }

//     // Flip perspective of drive overrides to be operator relative
//     int flipOperatorPerspectiveSign = 1;
//     if (Math.abs(RobotContainer.driveSubsystem.getOperatorForwardDirection().getDegrees()) > 90) {
//       flipOperatorPerspectiveSign = -1;
//     }

//     double xVelocity = isSetpointX
//         ? (flipOperatorPerspectiveSign * xPidController.calculate(currentPose.getX(), xSupplier.getAsDouble()))
//         : xSupplier.getAsDouble();
//     double yVelocity = isSetpointY
//         ? (flipOperatorPerspectiveSign * yPidController.calculate(currentPose.getY(), ySupplier.getAsDouble()))
//         : ySupplier.getAsDouble();
//     double thetaVelocity = isSetpointTheta
//         ? thetaPidController.calculate(currentPose.getRotation().getRadians(), thetaSupplier.get().getRadians())
//         : thetaSupplier.get().getRadians();

//     driveSubsystem.setControl(
//         driveRequest
//             .withDeadband(speedDeadband)
//             .withRotationalDeadband(rotationDeadband)
//             .withDriveRequestType(DriveRequestType.Velocity)
//             .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//             .withVelocityX(xVelocity)
//             .withVelocityY(yVelocity)
//             .withRotationalRate(thetaVelocity)
//     );

//     // ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

//     // Translation2d speeds = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
//     // speeds.rotateBy(driveSubsystem.getOperatorForwardDirection());

//     // fieldRelativeSpeeds.vxMetersPerSecond = speeds.getX();
//     // fieldRelativeSpeeds.vyMetersPerSecond = speeds.getY();

//     // ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//     //   fieldRelativeSpeeds,
//     //   driveSubsystem.getPose().getRotation()
//     // );

//     // SmartDashboard.putNumberArray("Targetspeeds", new Double[] {
//     //   targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond,
//     //   targetSpeeds.omegaRadiansPerSecond
//     // });

//     // // Note: it is important to not discretize speeds before or after
//     // // using the setpoint generator, as it will discretize them for you
//     // previousSetpoint = setpointGenerator.generateSetpoint(
//     //   previousSetpoint, // The previous setpoint
//     //   targetSpeeds, // The desired target speeds
//     //   0.02 // The loop time of the robot code, in seconds
//     // );

//     // ChassisSpeeds setpointGeneratedSpeeds = previousSetpoint.robotRelativeSpeeds();

//     // SmartDashboard.putNumberArray("s2", new Double[] {
//     //   setpointGeneratedSpeeds.vxMetersPerSecond, setpointGeneratedSpeeds.vyMetersPerSecond,
//     //   setpointGeneratedSpeeds.omegaRadiansPerSecond
//     // });

//     // // Deadband speeds
//     // if (Math.sqrt(setpointGeneratedSpeeds.vxMetersPerSecond * setpointGeneratedSpeeds.vxMetersPerSecond + setpointGeneratedSpeeds.vyMetersPerSecond * setpointGeneratedSpeeds.vyMetersPerSecond) < speedDeadband) {
//     //   setpointGeneratedSpeeds.vxMetersPerSecond = 0;
//     //   setpointGeneratedSpeeds.vyMetersPerSecond = 0;
//     // }
//     // if (Math.abs(setpointGeneratedSpeeds.omegaRadiansPerSecond) < rotationDeadband) {
//     //   setpointGeneratedSpeeds.omegaRadiansPerSecond = 0;
//     // }

//     // SmartDashboard.putNumberArray("SetpointSpeeds", new Double[] {
//     //   setpointGeneratedSpeeds.vxMetersPerSecond, setpointGeneratedSpeeds.vyMetersPerSecond,
//     //   setpointGeneratedSpeeds.omegaRadiansPerSecond
//     // });

//     // driveSubsystem.setControl(
//     //   driveRequest
//     //     .withDriveRequestType(DriveRequestType.Velocity)
//     //     .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//     //     .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesX())
//     //     .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesY())
//     //     .withSpeeds(setpointGeneratedSpeeds)
//     // );

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
