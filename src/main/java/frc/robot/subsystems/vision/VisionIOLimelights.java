package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.constants.RobotConstants;

public class VisionIOLimelights implements VisionIO{
//    NetworkTable turretTable = NetworkTableInstance.getDefault().getTable(RobotConstants.Vision.turretName);
//    NetworkTable leftTable = NetworkTableInstance.getDefault().getTable(RobotConstants.Vision.leftName);
//    NetworkTable rightTable = NetworkTableInstance.getDefault().getTable(RobotConstants.Vision.rightName);

    VisionIOInputs inputCache = new VisionIOInputs();

    public VisionIOLimelights(){
        setLLSettings(inputCache);
    }

    private void setLLSettings(VisionIOInputs inputs) {
        try {
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.leftName, "camerapose_robotspace_set", RobotConstants.Vision.leftPose);
//            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.turretName, "camerapose_robotspace_set", RobotConstants.Vision.turretPose);
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.rightName, "camerapose_robotspace_set", RobotConstants.Vision.rightPose);

            LimelightHelpers.SetIMUMode(RobotConstants.Vision.leftName, 4);
//            LimelightHelpers.SetIMUMode(RobotConstants.Vision.turretName, 4);
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

//            LimelightHelpers.SetRobotOrientation(
//                    RobotConstants.Vision.turretName,
//                    inputs.robotYawDegrees,
//                    inputs.robotYawRateDegreesPerSecond,
//                    inputs.robotPitchDegrees,
//                    inputs.robotPitchRateDegreesPerSecond,
//                    inputs.robotRollDegrees,
//                    inputs.robotRollRateDegreesPerSecond
//            );

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
//        inputs.turretHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.turretName);
        inputs.rightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.rightName);

        if (inputs.leftHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.leftName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.leftName);
            inputs.leftPoseEstimateMT1 = PoseEstimateStruct.fromPoseEstimate(mt1);
            inputs.leftPoseEstimateMT2 = PoseEstimateStruct.fromPoseEstimate(mt2);
            inputs.leftFiducials = RawFiducialStruct.fromRawFiducial(mt1.rawFiducials);
        }

//        if (inputs.turretHasTarget) {
//            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.turretName);
//            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.turretName);
//            inputs.turretPoseEstimateMT1 = PoseEstimateStruct.fromPoseEstimate(mt1);
//            inputs.getTurretPoseEstimateMT2 = PoseEstimateStruct.fromPoseEstimate(mt2);
//            inputs.turretFiducials = RawFiducialStruct.fromRawFiducial(mt1.rawFiducials);
//        }

        if (inputs.rightHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.rightName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.rightName);
            inputs.rightPoseEstimateMT1 = PoseEstimateStruct.fromPoseEstimate(mt1);
            inputs.rightPoseEstimateMT2 = PoseEstimateStruct.fromPoseEstimate(mt2);
            inputs.rightFiducials = RawFiducialStruct.fromRawFiducial(mt1.rawFiducials);
        }
    }
}
