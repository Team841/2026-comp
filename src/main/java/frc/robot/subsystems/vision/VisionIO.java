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

        public LimelightHelpers.RawFiducial[] leftRawFiducial = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.RawFiducial[] rightRawFiducial = new LimelightHelpers.RawFiducial[]{};

        public LimelightHelpers.PoseEstimate leftPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate leftPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);
        public LimelightHelpers.PoseEstimate rightPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate rightPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public double robotYawDegrees;
        public double robotYawRateDegreesPerSecond;
        public double robotPitchDegrees;
        public double robotPitchRateDegreesPerSecond;
        public double robotRollDegrees;
        public double robotRollRateDegreesPerSecond;
    }

    void updateInputs(VisionIOInputs inputs);
}
