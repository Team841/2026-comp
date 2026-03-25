package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        // public boolean turretHasTarget;
        public boolean leftHasTarget;
        public boolean rightHasTarget;

        // public RawFiducialStruct[] turretFiducials = new RawFiducialStruct[0];
        public RawFiducialStruct[] leftFiducials = new RawFiducialStruct[0];
        public RawFiducialStruct[] rightFiducials = new RawFiducialStruct[0];

        // public PoseEstimateStruct turretPoseEstimateMT1 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);
        // public PoseEstimateStruct TurretPoseEstimateMT2 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);

        public PoseEstimateStruct leftPoseEstimateMT1 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);
        public PoseEstimateStruct leftPoseEstimateMT2 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);

        public PoseEstimateStruct rightPoseEstimateMT1 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);
        public PoseEstimateStruct rightPoseEstimateMT2 = new PoseEstimateStruct(Pose2d.kZero, 0, 0, 0, 0, 0, 0, 0);

        public double robotYawDegrees;
        public double robotYawRateDegreesPerSecond;
        public double robotPitchDegrees;
        public double robotPitchRateDegreesPerSecond;
        public double robotRollDegrees;
        public double robotRollRateDegreesPerSecond;
    }

    void updateInputs(VisionIOInputs inputs);
}
