// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.vendor.BlueRelativeTarget;

public class DriveCommands {
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 10.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = Controls.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          double omega = omegaSupplier.getAsDouble();

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? RobotContainer.state.getRotation().plus(Rotation2d.k180deg)
                      : RobotContainer.state.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = Controls.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(
              RobotContainer.state.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? RobotContainer.state.getRotation().plus(new Rotation2d(Math.PI))
                      : RobotContainer.state.getRotation()));
        },
        drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(RobotContainer.state.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = RobotContainer.state.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = RobotContainer.state.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command stopWithX(Drive drive) {
    return Commands.run(() -> drive.stopWithX(), drive);
  }

  /**
   * Flips pose automatically from blue alliance
   */
  public static Command autoToPose(Pose2d target) {
    var pose = new BlueRelativeTarget(target);
    return autoToFieldPose(() -> pose.getFieldRelative());
  }

  /**
  * Flips pose automatically from blue alliance
  */
  public static Command autoToTarget(BlueRelativeTarget target) {
    return autoToFieldPose(() -> target.getFieldRelative());
  }

  public static Command autoToTarget(BlueRelativeTarget target, Distance tolerance) {
    var state = RobotContainer.state;
    return autoToFieldPose(() -> target.getFieldRelative(), false, () -> false).onlyWhile(
        () -> state.getPose().getTranslation().getDistance(target.getFieldRelativePose().getTranslation()) >= tolerance
            .in(Meters));
  }

  public static Command autoToFieldPose(Supplier<APTarget> target) {
    return autoToFieldPose(target, false, () -> true);
  }

  public static Command autoToFieldPose(Supplier<APTarget> target, boolean limitSlew, BooleanSupplier canFinish) {
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Used to lightly smooth out intermediate steps
    SlewRateLimiter xLimiter = new SlewRateLimiter(CodeConstants.AUTO_SLEW_LIMIT);
    SlewRateLimiter yLimiter = new SlewRateLimiter(CodeConstants.AUTO_SLEW_LIMIT);

    var state = RobotContainer.state;
    var drive = RobotContainer.drive;

    ChassisSpeeds lastOutput = new ChassisSpeeds();

    return Commands.run(() -> {
      Pose2d pose = state.getPose();
      ChassisSpeeds actualSpeeds = state.getRobotVelocity();

      // Don't feed in actual robot velocity to avoid posibility of feedback loops
      // Actually consistent with how real motion profiles (effectively what autopilot
      // generates) should be created
      APResult output = state.autopilot().calculate(pose, lastOutput, target.get());

      double omega = angleController.calculate(
          RobotContainer.state.getRotation().getRadians(), output.targetAngle().getRadians());

      var fieldRelativeGoalSpeed = new ChassisSpeeds(
          output.vx().in(MetersPerSecond),
          output.vy().in(MetersPerSecond),
          omega);

      // Only limit slew within a path to smooth switchovers
      if (limitSlew && !canFinish.getAsBoolean()) {
        fieldRelativeGoalSpeed.vxMetersPerSecond = xLimiter.calculate(fieldRelativeGoalSpeed.vxMetersPerSecond);
        fieldRelativeGoalSpeed.vyMetersPerSecond = yLimiter.calculate(fieldRelativeGoalSpeed.vyMetersPerSecond);
      }

      var robotRelativeGoalSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeGoalSpeed, pose.getRotation());

      Logger.recordOutput("RobotState/Autopilot/Target", target.get().getReference());
      Logger.recordOutput("RobotState/Autopilot/Field Relative Goal Speeds", fieldRelativeGoalSpeed);

      var trackingError = actualSpeeds.minus(robotRelativeGoalSpeed);
      if (Math.hypot(trackingError.vxMetersPerSecond,
          trackingError.vyMetersPerSecond) > CodeConstants.AUTO_MAX_TRACKING_ERROR.in(Meters)) {
        copyChassisSpeeds(lastOutput, actualSpeeds);
      } else {
        copyChassisSpeeds(lastOutput, robotRelativeGoalSpeed);
      }

      Logger.recordOutput("RobotState/Autopilot/Tracking Error", trackingError);

      drive.runVelocity(robotRelativeGoalSpeed);
    }, RobotContainer.drive)
        .until(() -> state.autopilot().atTarget(state.getPose(), target.get()) && canFinish.getAsBoolean())
        .beforeStarting(() -> {
          angleController.reset(RobotContainer.state.getRotation().getRadians());
          var fieldRelativeSpeeds = state.getFieldVelocity();

          xLimiter.reset(fieldRelativeSpeeds.vxMetersPerSecond);
          yLimiter.reset(fieldRelativeSpeeds.vyMetersPerSecond);

          var robotRelativeSpeeds = state.getRobotVelocity();
          copyChassisSpeeds(lastOutput, robotRelativeSpeeds);
        })
        .finallyDo(() -> {
          if (target.get().getVelocity() > 0) {
            return;
          }
          RobotContainer.drive.stop();
        });
  }

  // Helper to mutate chassisSpeeds objects (mutated because local variables
  // lambda enclosing scopes must be effectively final)
  private static void copyChassisSpeeds(ChassisSpeeds target, ChassisSpeeds toCopy) {
    target.vxMetersPerSecond = toCopy.vxMetersPerSecond;
    target.vyMetersPerSecond = toCopy.vyMetersPerSecond;
    target.omegaRadiansPerSecond = toCopy.omegaRadiansPerSecond;
  }
}
