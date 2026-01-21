// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.test.WheelRadiusCharacterization;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.MechanismPoses;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.SimVisionIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;

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
  public static final Drive driveSubsystem;

  /* Virtual Subsystems */
  /*
   * - May control hardware (like coprocessors), and have state and or periodic
   * methods. 
   * - Cannot be required by commands, and execute periodic loops *before* other
   * subsystems
   * - Do not nessesarily require an IO layer if not interacting with inputs
   * - Must be initialized in the static initializer block in sequence to avoid ordering issues
   */
  public static final Vision vision;
  public static final Telemetry telemetry;
  public static final MechanismPoses mechanismPoses;
  public static final Lights lightsSubsystem;

  /* Commands */

  static {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        driveSubsystem = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new Vision(
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_L),
            new LimelightIO(VisionConstants.LIMELIGHT_NAME_R));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSubsystem = new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        vision = new Vision(
            new SimVisionIO(VisionConstants.LIMELIGHT_NAME_L,
                VisionConstants.LEFT_CAMERA_TRANSFORM,
                simState::getPose),
            new SimVisionIO(VisionConstants.LIMELIGHT_NAME_R,
                VisionConstants.RIGHT_CAMERA_TRANSFORM,
                simState::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        driveSubsystem = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {}
        );

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

    // Warmup pathplanner to reduce delay when dynamic pathing
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * Configures default commands
   */
  private void configureDefaultCommands() {
    // Swerve telemetry from odometry thread
    driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
        driveSubsystem,
        Controls::getDriveYRaw,
        Controls::getDriveXRaw,
        Controls::getDriveRotationRaw
    ));
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
      // autoChooser.addOption("OPP2 Lolipop", new OPP2Lolipop());
    }

    testChooser = new LoggedDashboardChooser<>("Test Chooser", buildTestChooser());
  }

  /**
   * Binds controls
   */
  private void configureBindings() {
    Controls.rightJoystick.button(9).onTrue(Commands.runOnce(() -> {

    }));
    // Use the back button to zero both elevator and pivot in sequence
    // Controls.operatorController.back().onTrue(new ZeroMechanisms());

    // Simulation
    if (RobotBase.isSimulation()) {
      //     Controls.simController.button(1).onTrue(
      //             Commands.runOnce(() -> telemetry.toggleIntakeSimLoaded())
      //     );
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
        DriveCommands.wheelRadiusCharacterization(driveSubsystem));
    chooser.addOption(
        "Drive Wheel Radius Characterization (Custom)",
        WheelRadiusCharacterization.GetCharacterizationCommand());
    chooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(driveSubsystem));
    chooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        "Drive SysId (Dynamic Forward)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
}
