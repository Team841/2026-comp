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
            RobotConstants.Vision.turretPose[0] = RobotConstants.Vision.turretCameraToTurretCenter * inputs.turretAngle.getCos();
            RobotConstants.Vision.turretPose[1] = RobotConstants.Vision.turretCameraToTurretCenter * inputs.turretAngle.getSin();
            RobotConstants.Vision.turretPose[5] = inputs.turretAngle.getDegrees();

            Logger.recordOutput("Vision/TurretCameraPose", RobotConstants.Vision.turretPose);
            
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.frontLeftName, "camerapose_robotspace_set", RobotConstants.Vision.frontLeftPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.backLeftName, "camerapose_robotspace_set", RobotConstants.Vision.backLeftPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.turretName, "camerapose_robotspace_set", RobotConstants.Vision.turretPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.frontRightName, "camerapose_robotspace_set", RobotConstants.Vision.frontRightPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.backRightName, "camerapose_robotspace_set", RobotConstants.Vision.backRightPose);

            LimelightHelpers.SetIMUMode(RobotConstants.Vision.frontLeftName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.backLeftName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.turretName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.frontRightName, 0);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.backRightName, 0);

            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.frontLeftName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.backLeftName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );
            
            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.turretName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.frontRightName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.backRightName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );
        } catch (Exception e) {
            System.out.println("Error setting limelight settings: " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.inputCache = inputs;
        setLLSettings(this.inputCache);

        inputs.frontLeftHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.frontLeftName);
        inputs.backLeftHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.backLeftName);
        inputs.turretHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.turretName);
        inputs.frontRightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.frontRightName);
        inputs.backRightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.backRightName);

        if (inputs.frontLeftHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.frontLeftName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.frontLeftName);
            inputs.frontLeftPoseEstimateMT1 = mt1;
            inputs.frontLeftPoseEstimateMT2 = mt2;
            inputs.frontLeftRawFiducial = mt1.rawFiducials();
        }

        if (inputs.backLeftHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.backLeftName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.backLeftName);
            inputs.backLeftPoseEstimateMT1 = mt1;
            inputs.backLeftPoseEstimateMT2 = mt2;
            inputs.backLeftRawFiducial = mt1.rawFiducials();
        }

       if (inputs.turretHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.turretName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.turretName);
            inputs.turretPoseEstimateMT1 = mt1;
            inputs.turretPoseEstimateMT2 = mt2;
            inputs.turretRawFiducial = mt1.rawFiducials();
       }

        if (inputs.frontRightHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.frontRightName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.frontRightName);
            inputs.frontRightPoseEstimateMT1 = mt1;
            inputs.frontRightPoseEstimateMT2 = mt2;
            inputs.frontRightRawFiducial = mt1.rawFiducials();
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
