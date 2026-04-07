package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean turretHasTarget;
        public boolean frontLeftHasTarget;
        public boolean frontRightHasTarget;
        public boolean backLeftHasTarget;
        public boolean backRightHasTarget;

        public LimelightHelpers.RawFiducial[] frontLeftRawFiducial = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.RawFiducial[] frontRightRawFiducial = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.RawFiducial[] backLeftRawFiducial = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.RawFiducial[] backRightRawFiducial = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.RawFiducial[] turretRawFiducial = new LimelightHelpers.RawFiducial[]{};

        public LimelightHelpers.PoseEstimate frontLeftPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate frontLeftPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public LimelightHelpers.PoseEstimate frontRightPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate frontRightPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public LimelightHelpers.PoseEstimate backLeftPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate backLeftPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public LimelightHelpers.PoseEstimate backRightPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate backRightPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public LimelightHelpers.PoseEstimate turretPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate turretPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public double robotYawDegrees;
        public double robotYawRateDegreesPerSecond;
        public double robotPitchDegrees;
        public double robotPitchRateDegreesPerSecond;
        public double robotRollDegrees;
        public double robotRollRateDegreesPerSecond;

        public Rotation2d turretAngle;
    }

    void updateInputs(VisionIOInputs inputs);
}
