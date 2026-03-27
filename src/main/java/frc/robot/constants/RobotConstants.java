package frc.robot.constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotConstants {

    public static BooleanSupplier isRedAlliance = () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    };
    
    public static class Vision {

        public static final String leftName = "limelight-left";
        public static final String turretName = "limelight-turret";
        public static final String rightName = "limelight-right";

        public static final double[] leftPose = {0, -0.305, 0.53467, -12, 0, 90};
        public static final double[] rightPose = {0, 0.305, 0.53467, 12, 0, 270};
        public static final double[] turretPose = {0, 0, 0.7406386, 0, 0, 0};
    }

    public static class AutoAim {

        public static final double fieldLengthX = 16.54175;
        
        public static final Pose2d blueHubPose = new Pose2d(4.626, 4.035, new Rotation2d());
        public static final Pose2d redHubPose = new Pose2d(11.914, 4.035, new Rotation2d());

        public static final Pose2d bluePassShotHighPose = new Pose2d(3, 5.4, new Rotation2d());
        public static final Pose2d bluePassShotLowPose = new Pose2d(3, 2.6, new Rotation2d());

        public static final Pose2d redPassShotHighPose = new Pose2d(13.54175, 5.4, new Rotation2d());
        public static final Pose2d redPassShotLowPose = new Pose2d(13.54175, 2.6, new Rotation2d());
    }
}