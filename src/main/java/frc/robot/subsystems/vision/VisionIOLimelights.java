package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;
import frc.robot.constants.RobotConstants;

public class VisionIOLimelights implements VisionIO{
    VisionIOInputs inputCache = new VisionIOInputs();

    public VisionIOLimelights(){
        setLLSettings(inputCache);
    }

    private void setLLSettings(VisionIOInputs inputs) {
        try {
            RobotConstants.Vision.turretPose[0] = (RobotConstants.Vision.turretCameraToTurretCenter * inputs.turretAngle.getCos()) - 0.0127;
            RobotConstants.Vision.turretPose[1] = RobotConstants.Vision.turretCameraToTurretCenter * inputs.turretAngle.getSin();
            RobotConstants.Vision.turretPose[5] = inputs.turretAngle.getDegrees();

            Logger.recordOutput("Vision/TurretCameraPose", RobotConstants.Vision.turretPose);
            
            LimelightHelpers.setCameraPose_RobotSpace(
                RobotConstants.Vision.turretName, 
                RobotConstants.Vision.turretPose[0], 
                RobotConstants.Vision.turretPose[1], 
                RobotConstants.Vision.turretPose[2], 
                RobotConstants.Vision.turretPose[3], 
                RobotConstants.Vision.turretPose[4], 
                RobotConstants.Vision.turretPose[5]);

            LimelightHelpers.setCameraPose_RobotSpace(
                RobotConstants.Vision.frontLeftName, 
                RobotConstants.Vision.frontLeftPose[0], 
                RobotConstants.Vision.frontLeftPose[1], 
                RobotConstants.Vision.frontLeftPose[2], 
                RobotConstants.Vision.frontLeftPose[3], 
                RobotConstants.Vision.frontLeftPose[4], 
                RobotConstants.Vision.frontLeftPose[5]);
    
            LimelightHelpers.setCameraPose_RobotSpace(
                RobotConstants.Vision.backRightName, 
                RobotConstants.Vision.backRightPose[0], 
                RobotConstants.Vision.backRightPose[1], 
                RobotConstants.Vision.backRightPose[2], 
                RobotConstants.Vision.backRightPose[3], 
                RobotConstants.Vision.backRightPose[4], 
                RobotConstants.Vision.backRightPose[5]);
    
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.frontLeftName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.turretName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.backRightName, 0);
    
            LimelightHelpers.SetRobotOrientation_NoFlush(
                    RobotConstants.Vision.frontLeftName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );
            
            LimelightHelpers.SetRobotOrientation_NoFlush(
                    RobotConstants.Vision.turretName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            LimelightHelpers.SetRobotOrientation_NoFlush(
                    RobotConstants.Vision.backRightName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            LimelightHelpers.Flush();
        } catch (Exception e) {
            System.out.println("Error setting limelight settings: " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.inputCache = inputs;
        setLLSettings(this.inputCache);

        inputs.frontLeftHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.frontLeftName);
        inputs.turretHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.turretName);
        inputs.backRightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.backRightName);

        if (inputs.frontLeftHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.frontLeftName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.frontLeftName);
            inputs.frontLeftPoseEstimateMT1 = mt1;
            inputs.frontLeftPoseEstimateMT2 = mt2;
            inputs.frontLeftRawFiducial = mt1.rawFiducials();
        }

        if (inputs.turretHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.turretName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.turretName);
            inputs.turretPoseEstimateMT1 = mt1;
            inputs.turretPoseEstimateMT2 = mt2;
            inputs.turretRawFiducial = mt1.rawFiducials();
        }

        if (inputs.backRightHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.backRightName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.backRightName);
            inputs.backRightPoseEstimateMT1 = mt1;
            inputs.backRightPoseEstimateMT2 = mt2;
            inputs.backRightRawFiducial = mt1.rawFiducials();
        }
    }
}
