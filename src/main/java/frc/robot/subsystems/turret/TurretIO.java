package frc.robot.subsystems.turret;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public LimelightHelpers.RawFiducial[] rawFiducials = new LimelightHelpers.RawFiducial[]{};
        public LimelightHelpers.PoseEstimate poseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate poseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);
        public boolean turretCameraHasTarget = false;


    }

    void updateInputs(TurretIOInputs inputs);
}
