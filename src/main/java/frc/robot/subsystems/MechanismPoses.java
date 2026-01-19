package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.utils.lib.subsystems.VirtualSubsystem;

public class MechanismPoses extends VirtualSubsystem {
    private Pose3d[] poseArray = new Pose3d[4];

    // Base transforms for each mechanism (relative to robot center)
    private static final Transform3d ELEVATOR_BASE = new Transform3d(
            new Translation3d(0.12065, 0, 0.06985), // Elevator is near the front of the robot
            new Rotation3d(0, 0, 0)
    );

    private static final Transform3d PIVOT_BASE = new Transform3d(
            new Translation3d(0.19685, -0.184345, 0.692425), // Pivot is at the same base as elevator
            new Rotation3d(0, 0, 0)
    );

    private static final Transform3d GROUND_INTAKE_BASE = new Transform3d(
            new Translation3d(-0.33655, 0, 0.254), // Pivot is at the same base as elevator
            new Rotation3d(0, 0, 0)
    );

    public MechanismPoses() {
        Logger.recordOutput("Telemetry/Zero", Pose3d.kZero);
    }

    @Override
    public void latePeriodic() {
        updateElevatorPoses();
        updatePivotPose();

        Logger.recordOutput("Telemetry/MechanismPoses", poseArray);
    }

    private void updateElevatorPoses() {
        double elevatorHeight = RobotContainer.superstructure.elevator.getElevatorPositionMeters();

        // Calculate stage positions
        double carriageHeight;
        double firstStageHeight;

        if (elevatorHeight <= ElevatorConstants.FIRST_STAGE_START_HEIGHT) {
            // Only carriage moves in first half
            carriageHeight = elevatorHeight + 0.0254;
            firstStageHeight = 0;
        } else {
            // First stage starts moving in second half
            double remainingHeight = elevatorHeight - ElevatorConstants.FIRST_STAGE_START_HEIGHT;
            firstStageHeight = remainingHeight;
            carriageHeight = ElevatorConstants.FIRST_STAGE_START_HEIGHT + remainingHeight + 0.0254;
        }

        // Create first stage pose
        Pose3d firstStagePose = new Pose3d(
                ELEVATOR_BASE.getTranslation().plus(new Translation3d(0, 0, firstStageHeight)),
                ELEVATOR_BASE.getRotation()
        );

        // Create carriage pose
        Pose3d carriagePose = new Pose3d(
                ELEVATOR_BASE.getTranslation().plus(new Translation3d(0, 0, carriageHeight)),
                ELEVATOR_BASE.getRotation()
        );

        // Publish poses directly
        // elevatorFirstStagePosePub.set(firstStagePose);
        // elevatorCarriagePosePub.set(carriagePose);
        poseArray[2] = firstStagePose;
        poseArray[3] = carriagePose;
    }

    private void updatePivotPose() {
        double pivotAngle = Math.toRadians(RobotContainer.superstructure.pivot.getPivotPosition());
        double elevatorHeight = RobotContainer.superstructure.elevator.getElevatorPositionMeters();

        // Create pivot pose - rotates around Y axis, moves up with elevator carriage
        Pose3d pivotPose = new Pose3d(
                PIVOT_BASE.getTranslation().plus(new Translation3d(0, 0, elevatorHeight)),
                new Rotation3d(0, -pivotAngle, 0).plus(PIVOT_BASE.getRotation())
        );

        // Publish pose directly
        // pivotPosePub.set(pivotPose);

        double groundIntakeAngle = Math.toRadians(RobotContainer.groundSuperstructure.pivot.getPivotDegrees());

        // Create ground intake pose - rotates around Y axis
        Pose3d groundIntakePose = new Pose3d(
                GROUND_INTAKE_BASE.getTranslation(),
                new Rotation3d(0, -groundIntakeAngle, 0).plus(GROUND_INTAKE_BASE.getRotation())
        );

        // groundIntakePosePub.set(groundIntakePose);
        poseArray[0] = pivotPose;
        poseArray[1] = groundIntakePose;
    }
}