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
            
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.leftName, "camerapose_robotspace_set", RobotConstants.Vision.leftPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.turretName, "camerapose_robotspace_set", RobotConstants.Vision.turretPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.rightName, "camerapose_robotspace_set", RobotConstants.Vision.rightPose);

            LimelightHelpers.SetIMUMode(RobotConstants.Vision.leftName, 4);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.turretName, 3);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.rightName, 4);

            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.leftName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );

            // LimelightHelpers.SetRobotOrientation(
            //         RobotConstants.Vision.turretName,
            //         inputs.robotYawDegrees,
            //         inputs.robotYawRateDegreesPerSecond,
            //         inputs.robotPitchDegrees,
            //         inputs.robotPitchRateDegreesPerSecond,
            //         inputs.robotRollDegrees,
            //         inputs.robotRollRateDegreesPerSecond
            // );
            
            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.turretName,
                    inputs.turretAngle.getDegrees(),
                    0,
                    0,
                    0,
                    0,
                    0
            );
            LimelightHelpers.SetRobotOrientation(
                    RobotConstants.Vision.rightName,
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

        inputs.leftHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.leftName);
        inputs.turretHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.turretName);
        inputs.rightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.rightName);

        if (inputs.leftHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.leftName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.leftName);
            inputs.leftPoseEstimateMT1 = mt1;
            inputs.leftPoseEstimateMT2 = mt2;
            inputs.leftRawFiducial = mt1.rawFiducials();
        }

       if (inputs.turretHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.turretName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.turretName);
            inputs.turretPoseEstimateMT1 = mt1;
            inputs.turretPoseEstimateMT2 = mt2;
            inputs.turretRawFiducial = mt1.rawFiducials();
       }

        if (inputs.rightHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.rightName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.rightName);
            inputs.rightPoseEstimateMT1 = mt1;
            inputs.rightPoseEstimateMT2 = mt2;
            inputs.rightRawFiducial = mt1.rawFiducials();
        }
    }
}
