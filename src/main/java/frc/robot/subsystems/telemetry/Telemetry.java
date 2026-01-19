package frc.robot.subsystems.telemetry;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.external.ConcurrentTimeInterpolatableBuffer;
import frc.robot.utils.hardware.DeferredRefresher;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class Telemetry extends VirtualSubsystem {
    /* Pathplanner data */
    // private final NetworkTable pathplannerTable = inst.getTable("PathPlanner");
    // StructPublisher<Pose2d> pathplannerCurrentPoseNT = pathplannerTable
    //     .getStructTopic("PPCurrentPose", Pose2d.struct).publish();
    // StructPublisher<Pose2d> pathplannerTargetPoseNT = pathplannerTable
    //     .getStructTopic("PPTargetPose", Pose2d.struct).publish();
    // StructArrayPublisher<Pose2d> pathplannerCurrentTrajectory = pathplannerTable
    //     .getStructArrayTopic("PPCurrentTrajectory", Pose2d.struct).publish();

    // private final Pose2d[] trajTypeArray = new Pose2d[0];

    /*                 */
    /* Other Variables */
    /*                 */

    public boolean manipulatorCoralSimLoaded = false;
    public boolean intakeSimLoaded = false;
    public boolean intakeHandoffSimLoaded = false;
    public boolean algeaSimLoaded = false;

    // private PowerDistribution powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);

    public final CoralTracking coralTracking = new CoralTracking();

    // CAN checking variables
    private CANStatus rioCanStatus = new CANStatus();

    private Trigger rioCanStatusTrigger = new Trigger(() -> {
        CANJNI.getCANStatus(rioCanStatus);

        return rioCanStatus.receiveErrorCount == 0 && rioCanStatus.transmitErrorCount == 0;
    }).debounce(2); // If error seen in last two seconds, report issue

    // Async CANivore bus status checking
    private CANBus CANivoreBus = new CANBus(CANIds.CANivoreName);
    DeferredRefresher<CANBusStatus> canivoreRefresher = new DeferredRefresher<>(
            "CANivore Status",
            CodeConstants.PERIODIC_LOOP_TIME,
            () -> CANivoreBus.getStatus()
    );
    private Trigger drivetrainCanStatusTrigger = new Trigger(() -> {
        var canStatus = canivoreRefresher.getLatestValue();
        if (canStatus.isPresent()) {
            return (canStatus.get().Status.isOK()
                    && canStatus.get().TEC == 0
                    && canStatus.get().REC == 0);
        }
        return false; // Error if not present
    }).debounce(2); // If error seen in last two seconds, report issue

    /*                 */
    /*  Alerts System  */
    /*                 */

    private final Alert canFaultDetected = new Alert("CAN fault detected [See Console]", AlertType.kError);
    private final Alert rioCanError = new Alert("RIO CAN bus error", AlertType.kError);
    private final Alert canivoreError = new Alert("CANivore bus error", AlertType.kError);
    private final Alert visionFaultDetected = new Alert("", AlertType.kError);
    // private final Alert driverControllerDisconnected = new Alert("Driver controller disconnected [port 0].", AlertType.kWarning);
    private final Alert leftJoystickDisconnected = new Alert("Left joystick disconnected [port 0].",
            AlertType.kWarning);
    private final Alert rightJoystickDisconnected = new Alert("Right joystick disconnected [port 1].",
            AlertType.kWarning);
    private final Alert operatorControllerDisconnected = new Alert("Operator controller disconnected [port 2].",
            AlertType.kWarning);

    /*                       */
    /*  Latency Compensation */
    /*                       */

    // Timestamps are in the timebase of Timer.getFPGATimestamp()
    private ConcurrentTimeInterpolatableBuffer<Pose2d> poseHistoryBuffer = ConcurrentTimeInterpolatableBuffer
            .createBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> yawVelocityHistoryBuffer = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(CodeConstants.TELEMETRY_LOOKBACK_TIME);

    /**
     * Construct a telemetry subsystem
     */
    public Telemetry() {
        // MaxSpeed = PhysicalConstants.maxSpeed;

        // Set override state once to avoid it sticking around after code reboots
        publishOperatorOverrideInfo();
    }

    @Override
    public void latePeriodic() {
        coralTracking.update();

        // Update controls warnings
        // driverControllerDisconnected.set(!Controls.driverController.isConnected());
        leftJoystickDisconnected.set(!Controls.leftJoystick.isConnected());
        rightJoystickDisconnected.set(!Controls.rightJoystick.isConnected());
        operatorControllerDisconnected.set(!Controls.operatorController.isConnected());

        // Check for CAN errors
        rioCanError.set(!rioCanStatusTrigger.getAsBoolean());
        canivoreError.set(!drivetrainCanStatusTrigger.getAsBoolean());

        // System.out.println(RobotContainer.canConfigFailed);
        if (Robot.canConfigFailed || !rioCanStatusTrigger.getAsBoolean()
                || !drivetrainCanStatusTrigger.getAsBoolean()) {
            canFaultDetected.set(true);
        } else {
            canFaultDetected.set(false);
        }

        // Less accurate than high hz odometry thread but probably good enough? 
        poseHistoryBuffer.addSample(Timer.getTimestamp(), RobotContainer.driveSubsystem.getPose());
        yawVelocityHistoryBuffer.addSample(Timer.getTimestamp(),
                RobotContainer.driveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
    }

    /**
     * Gets the robot pose at the given timestamp (FPGA timebase) 
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        return poseHistoryBuffer.getSample(timestamp);
    }

    /**
     * Gets the robot yaw velocity at the given timestamp (FPGA timebase) 
     */
    public Optional<Double> getYawVelocityAtTimestamp(double timestamp) {
        return yawVelocityHistoryBuffer.getSample(timestamp);
    }

    // /**
    //  * Publish data about power usage to networktables for monitoring
    //  */
    // public void publishPDHInfo() {
    //     try {
    //         busVoltage.set(powerDistributionHub.getVoltage());
    //         temperature.set(powerDistributionHub.getTemperature());
    //         currentDraw.set(powerDistributionHub.getTotalCurrent());
    //         powerDraw.set(powerDistributionHub.getTotalPower());
    //         energyUsage.set(powerDistributionHub.getTotalEnergy());
    //     } catch (Exception e) {
    //         DriverStation.reportWarning("PDH Firmware Failure.", false);
    //     }
    // }

    /**
     * Update the state of the operator override. Should only be called when changing the state
     */
    public void publishOperatorOverrideInfo() {
        Logger.recordOutput("Controls/Override Enabled", RobotContainer.isOperatorOverride);
    }

    /**
     * Update the scoring metrics with the latest alignment and scoring durations
     * @param alignmentTime Time it took to complete the alignment in seconds
     * @param totalScoringTime Time it took for the entire scoring operation in seconds
     */
    public void updateScoringMetrics(double alignmentTime, double totalScoringTime) {
        // Only publish to SmartDashboard for driver visibility
        Logger.recordOutput("Telemetry/Recent Alignment Time", alignmentTime);
        Logger.recordOutput("Telemetry/Recent Total Scoring Time", totalScoringTime);
    }

    /**
     * Record a scoring operation timing
     * @param alignmentTime Time it took for alignment
     * @param totalTime Total time for the scoring operation
     */
    public void recordScoringTime(double alignmentTime, double totalTime) {
        updateScoringMetrics(alignmentTime, totalTime);
    }

    /**
     * Indicate that there is a Vision fault
     */
    public void setVisionFault(boolean value) {
        visionFaultDetected.set(value);
        if (value) {
            ArrayList<String> details = new ArrayList<>();
            var vision = RobotContainer.vision;
            if (!vision.leftLimelight.isAlive()) {
                details.add(vision.leftLimelight.getName());
            }
            if (!vision.rightLimelight.isAlive()) {
                details.add(vision.rightLimelight.getName());
            }

            visionFaultDetected.setText("Vision fault detected [" + String.join(", ", details) + "]");
        }
    }

    public void toggleManipulatorCoralSimLoaded() {
        manipulatorCoralSimLoaded = !manipulatorCoralSimLoaded;
    }

    public void toggleIntakeSimLoaded() {
        intakeSimLoaded = !intakeSimLoaded;
    }

    public void toggleIntakeHandoffSimLoaded() {
        intakeHandoffSimLoaded = !intakeHandoffSimLoaded;
    }

    public void toggleAlgeaSimLoaded() {
        algeaSimLoaded = !algeaSimLoaded;
    }
}
