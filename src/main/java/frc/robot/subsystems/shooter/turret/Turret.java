// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.TurretConstants;
import frc.robot.utils.lib.subsystems.ExpandedSubsystem;

public class Turret extends ExpandedSubsystem {
  public static record TurretSetpoint(
      Rotation2d heading,
      double velocity
  ) {}

  public static enum TurretState {
    BRAKE,
    TRACK_FIELD_RELATIVE,
    TRACK_TURRET_RELATIVE
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  @AutoLogOutput(key = "Turret/State")
  private TurretState state;

  private Rotation2d goalHeading = Rotation2d.kZero;
  private double goalVelocity = 0;

  private TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(TurretConstants.MAX_VELOCITY, TurretConstants.MAX_ACCELERATION));
  private State profileState = new State();

  public Turret(TurretIO turretIO) {
    io = turretIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/Turret", inputs);
    RobotContainer.state.updateTurret(Timer.getTimestamp(), inputs.absolutePosition, inputs.velocity);
  }

  @Override
  public void latePeriodic() {
    // Run after commandscheduler so commands can set a target properly
    if (!DriverStation.isEnabled()) {
      profileState = new State(inputs.relativePosition, 0);
      return;
    }

    if (state == TurretState.BRAKE) {
      runSetpoint(0, 0);
      return;
    }

    Rotation2d robotRotation = RobotContainer.state.getRotation();
    double robotTheta = Units.radiansToRotations(RobotContainer.state.getRobotVelocity().omegaRadiansPerSecond);

    Rotation2d turretRelativeGoalHeading = goalHeading;
    double turretRelativeGoalVelocity = goalVelocity;

    if (state == TurretState.TRACK_FIELD_RELATIVE) {
      turretRelativeGoalHeading = goalHeading.minus(robotRotation);
      turretRelativeGoalVelocity = goalVelocity - robotTheta;
    }
    double chosenHeading = adjustSetpointForWrap(turretRelativeGoalHeading.getRotations());

    State goalState = new State(
        MathUtil.clamp(chosenHeading, TurretConstants.MIN_POSITION_ROTATIONS, TurretConstants.MAX_POSITION_ROTATIONS),
        turretRelativeGoalVelocity);

    Logger.recordOutput("Turret/MotionProfile/GoalHeading", goalState.position);
    Logger.recordOutput("Turret/MotionProfile/GoalVelocity", goalState.velocity);
    Logger.recordOutput("Turret/MotionProfile/ProfileHeading", profileState.position);
    Logger.recordOutput("Turret/MotionProfile/ProfileVelocity", profileState.velocity);

    profileState = profile.calculate(CodeConstants.PERIODIC_LOOP_TIME, profileState, goalState);

    runSetpoint(profileState.position, profileState.velocity);
  }

  // Can be rewritten later to handle different turret capabilities
  private double adjustSetpointForWrap(double rotationsFromCenter) {
    // We have two options the raw rotationsFromCenter or +/- 1 rotation.
    double alternative = rotationsFromCenter - 1.0;
    if (rotationsFromCenter < 0.0) {
      alternative = rotationsFromCenter + 1.0;
    }

    if (alternative < TurretConstants.MIN_POSITION_ROTATIONS || alternative > TurretConstants.MAX_POSITION_ROTATIONS) {
      return rotationsFromCenter;
    }

    if (Math.abs(getPosition() - alternative) < Math.abs(getPosition() - rotationsFromCenter)) {
      return alternative;
    }
    return rotationsFromCenter;
  }

  private double getAbsolutePosition() {
    return getPosition() % 1.0; // If absolute encoders are added later, just replace this with an input
  }

  private void runDutyCycle(double dutyCycle) {
    Logger.recordOutput("Turret/OutputDutyCycle", dutyCycle);
    io.runDutyCycle(dutyCycle);
  }

  private void runSetpoint(double position, double velocity) {
    Logger.recordOutput("Turret/OutputPosition", position);
    Logger.recordOutput("Turret/OutputVelocity", velocity);
    io.runSetpoint(position, velocity);
  }

  // Public API
  public void setTargetSetpoint(Rotation2d heading, double velocity) {
    goalHeading = heading;
    goalVelocity = velocity;
  }

  public TurretState getState() {
    return state;
  }

  public void setState(TurretState state) {
    if (this.state == state) {
      return;
    }
    profileState = new State(inputs.relativePosition, inputs.velocity);
    this.state = state;
  }

  public double getPosition() {
    return inputs.relativePosition;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public boolean atSetpoint(Rotation2d heading, double tolerancePos) {
    return Math.abs(heading.getRotations() - getAbsolutePosition()) < tolerancePos;
  }

  public boolean atSetpoint(Rotation2d heading, double velocity, double tolerancePos, double toleranceVel) {
    return Math.abs(heading.getRotations() - getAbsolutePosition()) < tolerancePos &&
        Math.abs(velocity - getVelocity()) < toleranceVel;
  }

  // Commands
  // public Command aimShotCommand() {
  // return run(
  // () -> {
  // // var params = ShotCalculator.getInstance().getParameters();
  // setTargetSetpoint(params.turretAngle(), params.turretVelocity());
  // setState(TurretState.TRACK_FIELD_RELATIVE);
  // }).withName("Aim Shot Command");
  // }

  public void runSetpoint(TurretSetpoint setpoint, boolean fieldRelative) {
    setTargetSetpoint(setpoint.heading(), setpoint.velocity());
    setState(fieldRelative ? TurretState.TRACK_FIELD_RELATIVE : TurretState.TRACK_TURRET_RELATIVE);
  }

  public Command runSetpointCommand(Supplier<TurretSetpoint> setpoint, boolean fieldRelative) {
    return run(
        () -> {
          runSetpoint(setpoint.get(), fieldRelative);
        }
    ).withName("Run Turret Setpoint");
  }
}
