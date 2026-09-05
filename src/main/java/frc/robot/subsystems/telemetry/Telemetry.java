package frc.robot.subsystems.telemetry;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.data.Ports;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.hardware.DeferredRefresher;
import frc.robot.utils.lib.EpochTimer;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class Telemetry extends VirtualSubsystem {
  /* Pathplanner data */
  // private final NetworkTable pathplannerTable = inst.getTable("PathPlanner");
  // StructPublisher<Pose2d> pathplannerCurrentPoseNT = pathplannerTable
  // .getStructTopic("PPCurrentPose", Pose2d.struct).publish();
  // StructPublisher<Pose2d> pathplannerTargetPoseNT = pathplannerTable
  // .getStructTopic("PPTargetPose", Pose2d.struct).publish();
  // StructArrayPublisher<Pose2d> pathplannerCurrentTrajectory = pathplannerTable
  // .getStructArrayTopic("PPCurrentTrajectory", Pose2d.struct).publish();

  // private final Pose2d[] trajTypeArray = new Pose2d[0];

  /*                 */
  /* Other Variables */
  /*                 */

  // CAN checking variables
  private CANStatus rioCanStatus = new CANStatus();

  private Trigger rioCanStatusTrigger = new Trigger(() -> {
    CANJNI.getCANStatus(rioCanStatus);

    return rioCanStatus.receiveErrorCount == 0 && rioCanStatus.transmitErrorCount == 0;
  }).debounce(2); // If error seen in last two seconds, report issue

  // Async CANivore bus status checking
  private CANBus CANivoreBus = Ports.Bus.CANIVORE;
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
  /* Alerts System */
  /*                 */

  private final Alert canFaultDetected = new Alert("CAN fault detected [See Console]", AlertType.kError);
  private final Alert rioCanError = new Alert("RIO CAN bus error", AlertType.kError);
  private final Alert canivoreError = new Alert("CANivore bus error", AlertType.kError);
  private final Alert visionFaultDetected = new Alert("", AlertType.kError);
  // private final Alert driverControllerDisconnected = new Alert("Driver
  // controller disconnected [port 0].", AlertType.kWarning);
  private final Alert leftJoystickDisconnected = new Alert("Left joystick disconnected [port 0].",
      AlertType.kWarning);
  private final Alert rightJoystickDisconnected = new Alert("Right joystick disconnected [port 1].",
      AlertType.kWarning);
  private final Alert operatorControllerDisconnected = new Alert("Operator controller disconnected [port 2].",
      AlertType.kWarning);

  // Dashboard pose test
  public final Field2d dashboardField = new Field2d();
  // public final NetworkTable dashboardPoseTable = NetworkTableInstance.getDefault().getTable("DashboardPose");
  // private final DoubleArrayPublisher fieldPub = dashboardPoseTable.getDoubleArrayTopic("robotPose").publish();
  // private final StringPublisher fieldTypePub = dashboardPoseTable.getStringTopic(".type").publish();

  /**
   * Construct a telemetry subsystem
   */
  public Telemetry() {
    SmartDashboard.putData("Field", dashboardField);
  }

  @Override
  public void latePeriodic() {
    EpochTimer.BeginEpoch("Telemetry");
    {
      // Update controls warnings
      // driverControllerDisconnected.set(!Controls.driverController.isConnected());
      leftJoystickDisconnected.set(!Controls.leftJoystick.isConnected());
      rightJoystickDisconnected.set(!Controls.rightJoystick.isConnected());
      operatorControllerDisconnected.set(!Controls.operatorController.isConnected());

      // Check for CAN errors
      rioCanError.set(!rioCanStatusTrigger.getAsBoolean());
      canivoreError.set(!drivetrainCanStatusTrigger.getAsBoolean());

      if (Robot.canConfigFailed || !rioCanStatusTrigger.getAsBoolean()
          || !drivetrainCanStatusTrigger.getAsBoolean()) {
        canFaultDetected.set(true);
      } else {
        canFaultDetected.set(false);
      }

      checkVisionFault();

      var pose = RobotContainer.state.getPose();
      dashboardField.setRobotPose(pose);
      // fieldTypePub.set("Field2d");
      // fieldPub.set(new double[] {
      //     pose.getX(),
      //     pose.getY(),
      //     pose.getRotation().getDegrees()
      // });
    }
    EpochTimer.EndEpoch("Telemetry");
  }

  /**
   * Indicate that there is a Vision fault
   */
  public void checkVisionFault() {
    var vision = RobotContainer.vision;

    ArrayList<String> details = new ArrayList<>();
    boolean visionOk = true;

    if (!vision.frameCamera.isAlive()) {
      details.add(vision.frameCamera.getName());
      visionOk = false;
    }
    if (!vision.turretCamera.isAlive()) {
      details.add(vision.turretCamera.getName());
      visionOk = false;
    }

    visionFaultDetected.set(!visionOk);
    if (!visionOk) {
      visionFaultDetected.setText("Vision fault detected [" + String.join(", ", details) + "]");
    }
  }
}
