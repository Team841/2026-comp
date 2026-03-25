package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean turretHasTarget;
        public boolean leftHasTarget;
        public boolean rightHasTarget;

        public RawFiducialStruct[] turretFiducials;
        public RawFiducialStruct[] leftFiducials;
        public RawFiducialStruct[] rightFiducials;

        public PoseEstimateStruct turretPoseEstimateMT1;
        public PoseEstimateStruct getTurretPoseEstimateMT2;

        public PoseEstimateStruct leftPoseEstimateMT1;
        public PoseEstimateStruct leftPoseEstimateMT2;

        public PoseEstimateStruct rightPoseEstimateMT1;
        public PoseEstimateStruct rightPoseEstimateMT2;

        public double robotYawDegrees;
        public double robotYawRateDegreesPerSecond;
        public double robotPitchDegrees;
        public double robotPitchRateDegreesPerSecond;
        public double robotRollDegrees;
        public double robotRollRateDegreesPerSecond;
    }

    void updateInputs(VisionIOInputs inputs);
}
