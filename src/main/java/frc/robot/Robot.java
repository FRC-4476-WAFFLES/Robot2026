// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.adaptable.AdaptableManager;
import frc.robot.data.BuildConstants;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.Mode;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.subsystems.ExpandedSubsystemManager;
import frc.robot.utils.vendor.FuelSim;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;

  private final RobotContainer m_robotContainer;

  public static boolean canConfigFailed = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // AdvantageKit Init
    Logger.recordMetadata("ProjectName", "Robot2026"); // Set a metadata value

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        break;

      case SIM:
        // Running a physics simulator. NT4 matches what REAL publishes, so
        // AdvantageScope connects the same way at a desk as it does at an event.
        // RLOG is kept as a lower-latency alternative. The WPILOG file is what
        // tests and agents read back afterwards.
        Logger.addDataReceiver(new WPILOGWriter("simlogs/"));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new RLOGServer());

        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Log metadata about the build
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Deployed Branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Build Timestamp", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Repository", BuildConstants.MAVEN_NAME);

    // Force load map early
    System.out.println("Loading Apriltag Map: " + VisionConstants.APRIL_TAG_FIELD_LAYOUT.toString());
    Logger.recordMetadata("Apriltag Map", VisionConstants.APRITL_TAG_MAP_NAME);

    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    System.out.println("Logger initialized. Robot program starting...");

    // LaserCAN stuff
    // CanBridge.runTCP();

    // Use realtime thread priority. This is dangerous and may have consequences for
    // other threads ie. networktables
    // Threads.setCurrentThreadPriority(true, 10);

    // Lower brownout threshold from default (6.75V) to give more headroom before
    // outputs cut out. Only applies on real hardware — RIO 1 ignores this call.
    if (Constants.getMode() == Mode.REAL) {
      RobotController.setBrownoutVoltage(6.5);
    }

    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer();

    // Log when commands start
    CommandScheduler.getInstance().onCommandInitialize(
        command -> Logger.recordOutput("Commands/" + command.getName(), true)
    );

    // Log when commands end or are interrupted
    CommandScheduler.getInstance().onCommandFinish(
        command -> Logger.recordOutput("Commands/" + command.getName(), false)
    );

    CommandScheduler.getInstance().onCommandInterrupt(
        command -> Logger.recordOutput("Commands/" + command.getName(), false)
    );

    if (CodeConstants.MANUAL_SHOOTER_TUNING) {
      // Quick & dirty
      SmartDashboard.putNumber("Shooter Speed", 0);
      SmartDashboard.putNumber("Hood Angle", 0);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions
   */
  @Override
  public void robotPeriodic() {
    // Time full loop refresh rate including waiting / background tasks that might
    // affect loop stability
    EpochTimer.EndEpoch("RobotRefresh");
    EpochTimer.BeginEpoch("RobotRefresh");

    // Refresh CAN signals from CTRE devices
    EpochTimer.BeginEpoch("PhoenixRefresh");
    {
      PhoenixHelpers.refreshAllSignals();
    }
    EpochTimer.EndEpoch("PhoenixRefresh");

    // Periodic Loop
    EpochTimer.BeginEpoch("Periodic");
    {
      RobotContainer.state.updateEnabledState();

      // Runs software subsystem early periodic methods
      ExpandedSubsystemManager.RunEarlyPeriodic();

      // Runs the Scheduler. This is responsible for polling buttons, adding
      // newly-scheduled
      // commands, running already-scheduled commands, removing finished or
      // interrupted commands,
      // and running subsystem periodic() methods. This must be called from the
      // robot's periodic
      // block in order for anything in the Command-based framework to work.
      EpochTimer.BeginEpoch("CommandScheduler");
      CommandScheduler.getInstance().run();
      EpochTimer.EndEpoch("CommandScheduler");

      // Runs software subsystem late periodic methods
      ExpandedSubsystemManager.RunLatePeriodic();
    }
    EpochTimer.EndEpoch("Periodic");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.gc();

    // Ensure both auto and test commands are canceled
    cancelControllingCommands();

    // Disable controller vibration in case disabled while rumbling
    Controls.operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    RobotContainer.state.setRumbleOperator(false);
  }

  @Override
  public void disabledPeriodic() {
    AdaptableManager.periodic();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Ensure a clean slate before starting auto
    cancelControllingCommands();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // Ensure both auto and test commands are canceled
    cancelControllingCommands();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_testCommand = m_robotContainer.getTestCommand();

    if (m_testCommand != null) {
      m_testCommand.schedule();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // The command scheduler will automatically run the test commands
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  @SuppressWarnings("unused")
  public void simulationPeriodic() {
    if (Constants.getMode() == Mode.SIM && CodeConstants.USE_FUEL_SIMULATION) {
      FuelSim.getInstance().updateSim();
    }
  }

  /** Cancels both the auto and test commands if present */
  private void cancelControllingCommands() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (m_testCommand != null) {
      m_testCommand.cancel();
    }
  }

  /**
   * Indicate that there is a CAN fault in configuring devices
   */
  public static void setCANConfigErrorFlag() {
    canConfigFailed = true;
  }
}
