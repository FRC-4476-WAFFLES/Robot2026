// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.AutoWinnerOverride;
import frc.robot.autos.Left;
import frc.robot.autos.LeftDepot;
import frc.robot.autos.LeftGreedy;
import frc.robot.autos.Preload;
import frc.robot.autos.adaptable.Adaptable;
import frc.robot.autos.adaptable.AutoVisualizer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.commands.test.WheelRadiusCharacterization;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.CodeConstants.ManualOverrideTarget;
import frc.robot.data.Constants.ExpanderConstants;
import frc.robot.data.Constants.ExpanderConstants.ExpanderPosition;
import frc.robot.data.Constants.IntakeConstants;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.MechanismPoses;
import frc.robot.subsystems.StateOrchestrator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ExpanderState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.ShotPlanner;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretSetpoint;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.SimVisionIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.utils.vendor.FuelSim;
import frc.robot.utils.vendor.HubShiftUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Global Robot State */
  public static LoggedDashboardChooser<Command> autoChooser;
  public static LoggedDashboardChooser<Command> testChooser;

  public static RobotState state = new RobotState();
  public static SimState simState = (Constants.getMode() == Mode.SIM) ? new SimState() : null;

  /* Hardware Subsystems */
  /* Require an IO layer if receiving any inputs */
  public static final Drive drive;
  public static final Turret turret;
  public static final Intake intake;
  public static final Hood hood;
  public static final Indexer indexer;
  public static final Flywheel flywheel;
  public static final Climber climber;

  /* Virtual Subsystems */
  /*
   * - May control hardware (like coprocessors), and have state and or periodic
   * methods.
   * - Cannot be required by commands, and execute periodic loops *before* other
   * subsystems
   * - Do not nessesarily require an IO layer if not interacting with inputs
   * - Must be initialized in the static initializer block in sequence to avoid
   * ordering issues
   */
  public static final Vision vision;
  public static final Telemetry telemetry;
  public static final MechanismPoses mechanismPoses;
  public static final Lights lightsSubsystem;
  public static final StateOrchestrator stateOrchestrator;

  private final Alert autoWinnerNotSet = new Alert("AUTO WINNER NOT SET", AlertType.kError);

  static {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        turret = new Turret(new TurretIOTalonFX());

        hood = new Hood(new HoodIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX() {});

        indexer = new Indexer(new IndexerIOTalonFX());

        flywheel = new Flywheel(new FlywheelIOTalonFX());

        climber = new Climber(new ClimberIO() {}); // No climber on robot yet

        vision = new Vision(
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_FRAME),
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_TURRET));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        turret = new Turret(new TurretIOSim());

        hood = new Hood(new HoodIOSim());

        intake = new Intake(new IntakeIOSim());

        indexer = new Indexer(new IndexerIOSim());

        flywheel = new Flywheel(new FlywheelIOSim());

        climber = new Climber(new ClimberIOSim());

        if (CodeConstants.USE_VISION_SIMULATION) {
          vision = new Vision(
              new SimVisionIO(VisionConstants.LIMELIGHT_NAME_FRAME,
                  Transform3d.kZero,
                  simState::getPose),
              new SimVisionIO(VisionConstants.LIMELIGHT_NAME_TURRET,
                  Transform3d.kZero,
                  simState::getPose));
        } else {
          vision = new Vision(
              new VisionIO() {},
              new VisionIO() {}
          );
        }

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {}
        );

        turret = new Turret(new TurretIO() {});

        hood = new Hood(new HoodIO() {});

        intake = new Intake(new IntakeIO() {});

        indexer = new Indexer(new IndexerIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});

        climber = new Climber(new ClimberIO() {});

        vision = new Vision(
            new VisionIO() {},
            new VisionIO() {}
        );

        break;
    }

    // lightsSubsystem = new Lights(new LightIOCandle()); // Just always use real io
    lightsSubsystem = null;
    // Subsystems with no IO
    telemetry = new Telemetry();
    mechanismPoses = new MechanismPoses();
    stateOrchestrator = new StateOrchestrator();
  }

  /**
   * The static entry point for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  @SuppressWarnings("unused")
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    registerNamedCommands();
    configureCommandChoosers();

    if (Constants.getMode() == Mode.SIM && CodeConstants.USE_FUEL_SIMULATION) {
      configureFuelSim();
    }

    HubShiftUtil.setAllianceWinOverride(
        () -> {
          if (state.autoWinnerOverride() == AutoWinnerOverride.THEM) {
            return Optional.of(false);
          }
          if (state.autoWinnerOverride() == AutoWinnerOverride.US) {
            return Optional.of(true);
          }
          return Optional.empty();
        });

    SmartDashboard.putBoolean("Manual", false);

    // Force static initialization
    ShotPlanner.aimManual();
    // Warmup pathplanner to reduce delay when dynamic pathing
    // FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * Configures default commands
   */
  private void configureDefaultCommands() {
    // Swerve telemetry from odometry thread
    drive.setDefaultCommand(DriveCommands.joystickDrive(
        drive,
        Controls::getDriveYRaw,
        Controls::getDriveXRaw,
        Controls::getDriveRotationRaw
    ));

    // drive.setDefaultCommand(DriveCommands.testDrive(
    // drive,
    // Controls::getDriveYRaw,
    // Controls::getDriveXRaw,
    // Controls::getDriveRotationRaw
    // ));

    // Testing
    // turret.setDefaultCommand(turret.runSetpointCommand(() -> Rotation2d.k180deg,
    // () -> 0, true));
  }

  /**
   * Configures test & auto choosers
   */
  private void configureCommandChoosers() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    if (CodeConstants.USE_PATHPLANNER_AUTOS) {
      autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    } else {
      autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
      autoChooser.addDefaultOption("None", Commands.none());
      autoChooser.addOption("Preload", new Preload());
      autoChooser.addOption("Left Depot", new LeftDepot());
      autoChooser.addOption("Left", new Left());
      autoChooser.addOption("Left Greedy", new LeftGreedy());
      autoChooser.addOption("Adaptable", Adaptable.run());
      autoChooser.onChange(cmd -> {
        if (autoChooser.getSendableChooser().getSelected() == "Adaptable") { // Scuffed and almost certainly not replay
                                                                             // compatible
          Adaptable.InvalidateCache();
        } else {
          AutoVisualizer.ClearVisualizer();
        }
      });
    }

    testChooser = new LoggedDashboardChooser<>("Test Chooser", buildTestChooser());
  }

  /**
   * Binds controls
   */
  private void configureBindings() {
    Controls.rightJoystick.button(9).onTrue(Commands.runOnce(() -> {
      drive.resetGyro();
    }));

    // Bottom face button
    Controls.rightJoystick.button(2).onTrue(Commands.runOnce(() -> state.toggleManualMode()));

    // Right face button
    // Manually toggles intake
    Controls.leftJoystick.button(4).onTrue(intake.toggleExtended());

    // Pressing in any capacity will extend intake
    // Intake rollers run while pressed
    Controls.leftJoystick.button(1).whileTrue(IntakeCommands.intakeCommand());

    Controls.operatorController.leftBumper()
        .whileTrue(Commands.startEnd(() -> state.setOuttakeDesired(true), () -> state.setOuttakeDesired(false)));

    // Intake
    state.expanderStowed().whileTrue(Commands.run(
        () -> intake.setExpanderSetpoint(ExpanderPosition.STOWED)
    ).withName("IntakeStowed"));
    state.expanderExtended().whileTrue(Commands.run(
        () -> intake.setExpanderSetpoint(ExpanderPosition.EXTENDED)
    ).withName("IntakeExtended"));
    state.expanderIntaking().whileTrue(Commands.run(
        () -> intake.setExpanderSetpoint(ExpanderPosition.INTAKING)
    ).withName("IntakeIntaking"));
    Timer agitationTimer = new Timer();
    state.expanderAgitating().whileTrue(Commands.run(
        () -> {
          if (state.getExpanderState() == ExpanderState.FULLY_AGITATING ||
              (state.isForceIntakeIn() && state.autonomousEnabled())) {
            intake.setExpanderSetpoint(ExpanderPosition.STOWED);
          } else {
            if (agitationTimer.get() % ExpanderConstants.AGITATION_CYCLE_TIME < 0.5) {
              intake.setExpanderSetpoint(ExpanderPosition.AGITATION_MAX);
            } else {
              intake.setExpanderSetpoint(ExpanderPosition.EXTENDED);
            }
          }
        }
    ).beforeStarting(() -> agitationTimer.restart()).withName("IntakeAgitation"));

    var fullMotionAgitation = Controls.operatorController.leftTrigger();
    state.shouldAgitate()
        .whileTrue(Commands.runEnd(
            () -> {
              if (fullMotionAgitation.getAsBoolean()) {
                state.setExpanderState(ExpanderState.FULLY_AGITATING);
              } else {
                state.setExpanderState(ExpanderState.AGITATING);
              }
              intake.setIntakeDutyCycle(IntakeConstants.AGITATION_DUTY_CYCLE);
            },
            () -> {
              intake.setIntakeDutyCycle(0);
              state.setExpanderState(ExpanderState.EXTENDED);
            }
        ));

    state.shouldIntake().whileTrue(Commands.runEnd(
        () -> {
          intake.setIntakeDutyCycle(IntakeConstants.INTAKE_DUTY_CYCLE);
          state.setExpanderState(ExpanderState.INTAKING);
        },
        () -> {
          intake.setIntakeDutyCycle(0);
          state.setExpanderState(ExpanderState.EXTENDED);
        }, intake
    ));

    state.shouldOuttake()
        .whileTrue(
            Commands.runEnd(
                () -> intake.setIntakeDutyCycle(IntakeConstants.OUTTAKE_DUTY_CYCLE),
                () -> intake.setIntakeDutyCycle(0),
                intake).withName("Outtake"));

    // Passing mode
    state.shooterTargetPassing().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimToPass();
      turret.runSetpoint(parms.turretSetpoint(), true);
      flywheel.runSetpoint(Controls.shootButton.getAsBoolean() ? parms.flywheelSpeed() : 0);
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Pass"));

    // Hub mode
    state.shooterTargetsHub().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimToHub();

      var turretSetpoint = parms.turretSetpoint();
      if (state.onBump) {
        var chosenOffset = ShotPlanner.getTurretBumpOffset();
        Logger.recordOutput("Turret/Bump Offset", chosenOffset);
        turret.runSetpoint(new TurretSetpoint(turretSetpoint.heading().plus(chosenOffset),
            turretSetpoint.velocity()), true);
        flywheel.runSetpoint(parms.flywheelSpeed() + 2);
      } else {
        turret.runSetpoint(parms.turretSetpoint(), true);
        flywheel.runSetpoint(parms.flywheelSpeed());
      }
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Hub"));

    // Aim at tag after crossing bump
    state.shooterTargetTag().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimToTag();
      turret.runSetpoint(parms.turretSetpoint(), true);
      flywheel.runSetpoint(Controls.shootButton.getAsBoolean() ? parms.flywheelSpeed() : 0);
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Tag"));

    state.shooterHandleBeached().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimBeached();
      turret.runSetpoint(parms.turretSetpoint(), false);
      flywheel.runSetpoint(parms.flywheelSpeed());
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Handle Beached"));

    state.shouldFire().whileTrue(ShooterCommands.shootCommand()).onFalse(
        ShooterCommands.backoffIndexer()
    );

    // Manual mode
    state.manualMode().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimManual();
      turret.runSetpoint(parms.turretSetpoint(), false);
      flywheel.runSetpoint(parms.flywheelSpeed());
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Manual Aim"));
    state.shouldFireManual().whileTrue(ShooterCommands.shootCommand()).onFalse(
        ShooterCommands.backoffIndexer()
    );

    // Auto winner override
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));

    Controls.operatorController.povUp()
        .onTrue(Commands.runOnce(() -> state.setAutoWinnerOverride(AutoWinnerOverride.THEM)));
    Controls.operatorController.povDown()
        .onTrue(Commands.runOnce(() -> state.setAutoWinnerOverride(AutoWinnerOverride.US)));

    // Warnings about auto winner not being set
    Timer teleopElapsedTimer = new Timer();
    RobotModeTriggers.teleop()
        .onTrue(
            Commands.runOnce(
                () -> {
                  teleopElapsedTimer.restart();
                }));
    RobotModeTriggers.teleop()
        .and(() -> !(DriverStation.getGameSpecificMessage().length() > 0))
        .and(() -> HubShiftUtil.getAllianceWinOverride().isEmpty())
        .and(() -> teleopElapsedTimer.hasElapsed(1.0) && CodeConstants.LIMIT_TO_HUB_SHIFTS)
        .whileTrue(
            Commands.runEnd(
                () -> {
                  Controls.operatorController.setRumble(RumbleType.kBothRumble, 1);
                },
                () -> {
                  Controls.operatorController.setRumble(RumbleType.kBothRumble, 0);
                }))
        .whileTrue(
            Commands.startEnd(() -> autoWinnerNotSet.set(true), () -> autoWinnerNotSet.set(false)));

    // Overrides - Operator Controller
    // Temporarily disabledw
    // Controls.operatorController.y()
    // .onTrue(Commands.runOnce(() ->
    // state.setManualOverrideTarget(ManualOverrideTarget.PASS)));
    // Controls.operatorController.b()
    // .onTrue(Commands.runOnce(() ->
    // state.setManualOverrideTarget(ManualOverrideTarget.TRENCH)));
    Controls.operatorController.a()
        .onTrue(Commands.runOnce(() -> state.setManualOverrideTarget(ManualOverrideTarget.FRONT_CLOSE)));

    // Simulation
    if (RobotBase.isSimulation()) {
      // Controls.simController.button(1).onTrue(
      // Commands.runOnce(() -> telemetry.toggleIntakeSimLoaded())
      // );
    }
  }

  /**
   * Use this method to define name->command mappings. Names will be used by
   * PathPlanner to
   * call commands in full autos.
   */
  private void registerNamedCommands() {
    // Register Named Commands
    // Add other commands to be able to run them in autos
    // NamedCommands.registerCommand("exampleCommand", exampleCommand);
  }

  /**
   * Use this method to define a list of commands that can be chosen from in test
   * mode
   */
  private SendableChooser<Command> buildTestChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();

    chooser.setDefaultOption("None", Commands.none());

    // Set up SysId routines
    chooser.addOption(
        "Drive Wheel Radius Characterization (AKit)",
        DriveCommands.wheelRadiusCharacterization(drive));
    chooser.addOption(
        "Drive Wheel Radius Characterization (Custom)",
        WheelRadiusCharacterization.GetCharacterizationCommand());
    chooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(drive));
    chooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        "Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    return chooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Use this to pass the testing command to the main {@link Robot} class.
   *
   * @return the command to run in testng mode
   */
  public Command getTestCommand() {
    return testChooser.get();
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        PhysicalConstants.FULL_WIDTH.in(Meters),
        PhysicalConstants.FULL_LENGTH.in(Meters),
        PhysicalConstants.BUMPER_HEIGHT.in(Meters),
        state::getPose,
        state::getFieldVelocity);
    // Intake is on the physical front of the robot (+X side)
    instance.registerIntake(
        PhysicalConstants.FULL_LENGTH.div(2).in(Meters),
        PhysicalConstants.FULL_LENGTH.div(2).plus(Inches.of(7)).in(Meters),
        -PhysicalConstants.FULL_WIDTH.div(2).in(Meters),
        PhysicalConstants.FULL_WIDTH.div(2).in(Meters),
        () -> state.getExpanderState() == ExpanderState.EXTENDED,
        simState::simIntake);

    instance.start();
  }
}
