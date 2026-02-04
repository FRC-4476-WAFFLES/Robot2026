// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.TemplateAuto;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.commands.test.WheelRadiusCharacterization;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.MechanismPoses;
import frc.robot.subsystems.StateOrchestrator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.SimVisionIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.utils.vendor.FuelSim;

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
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> testChooser;

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

  /* Commands */

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

        intake = new Intake(new IntakeIOTalonFX());

        indexer = new Indexer(new IndexerIOTalonFX());

        flywheel = new Flywheel(new FlywheelIOTalonFX());

        climber = new Climber(new ClimberIOTalonFX());

        vision = new Vision(
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_L),
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_R));

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
              new SimVisionIO(VisionConstants.LIMELIGHT_NAME_L,
                  VisionConstants.LEFT_CAMERA_TRANSFORM,
                  simState::getPose),
              new SimVisionIO(VisionConstants.LIMELIGHT_NAME_R,
                  VisionConstants.RIGHT_CAMERA_TRANSFORM,
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

    // Subsystems with no IO
    telemetry = new Telemetry();
    mechanismPoses = new MechanismPoses();
    lightsSubsystem = new Lights();
    stateOrchestrator = new StateOrchestrator();
  }

  /**
   * The static entry point for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    registerNamedCommands();
    configureCommandChoosers();

    if (Constants.getMode() == Mode.SIM && CodeConstants.USE_FUEL_SIMULATION) {
      configureFuelSim();
    }

    // Warmup pathplanner to reduce delay when dynamic pathing
    FollowPathCommand.warmupCommand().schedule();
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
      autoChooser.addOption("Template", new TemplateAuto());
      // autoChooser.addOption("OPP2 Lolipop", new OPP2Lolipop());
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
    // Use the back button to zero both elevator and pivot in sequence
    // Controls.operatorController.back().onTrue(new ZeroMechanisms());
    Controls.rightJoystick.button(1).onTrue(Commands.runOnce(() -> {
      intake.setExpanderState(ExpanderState.EXTENDED);
    }));
    Controls.rightJoystick.button(1).onFalse(Commands.runOnce(() -> {
      intake.setExpanderState(ExpanderState.STOWED);
    }));
    Controls.rightJoystick.button(3).onTrue(climber.moveElevator(Constants.ClimberConstants.CLIMBER_ROTATIONS))
        .onFalse(climber.moveElevator(0));
    Controls.operatorController.a().onTrue(indexer.runIndexer(Constants.SpindexerConstants.TEST_VELOCITY))
        .onFalse(indexer.runIndexer(0));

    // Passing mode
    state.shooterTargetPassing().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimToPass();
      turret.runSetpoint(parms.turretSetpoint(), true);
      flywheel.runSetpoint(parms.flywheelSpeed());
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Pass"));

    // Hub mode
    state.shooterTargetsHub().whileTrue(Commands.run(() -> {
      var parms = ShotPlanner.aimToHub();
      turret.runSetpoint(parms.turretSetpoint(), true);
      flywheel.runSetpoint(parms.flywheelSpeed());
      hood.runSetpoint(parms.hoodAngle());
    }).withName("Shooter Hub"));

    state.shouldFire().whileTrue(ShooterCommands.shootCommand());

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
    instance.registerIntake(
        -PhysicalConstants.FULL_LENGTH.div(2).in(Meters),
        PhysicalConstants.FULL_LENGTH.div(2).in(Meters),
        -PhysicalConstants.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
        -PhysicalConstants.FULL_WIDTH.div(2).in(Meters),
        () -> intake.getExpanderState() == ExpanderState.EXTENDED,
        simState::simIntake);

    instance.start();
  }
}
