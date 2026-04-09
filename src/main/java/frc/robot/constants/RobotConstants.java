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

    public enum autoAimMode{
        HUB,
        OutpostCorner,
    }

    public static autoAimMode currentAimMode = autoAimMode.HUB;
    
    public static class Vision {

        public static final String backLeftName = "limelight-bleft";
        public static final String frontLeftName = "limelight-fleft";
        public static final String backRightName = "limelight-bright";
        public static final String frontRightName = "limelight-fright";
        public static final String turretName = "limelight-turret";

        public static final double[] backLeftPose = {-0.16, -0.26, 0.51, -12, 9, 127.5};
        public static final double[] backRightPose = {-0.16, 0.26, 0.51, 12, 9, -127.5};
        public static final double[] frontLeftPose = {-0.056, -0.26, 0.51, 9, 12, 52.5};
        public static final double[] frontRightPose = {-0.056, 0.26, 0.51, -9, 12, -52.5};
        public static double[] turretPose = {0, 0, 0, 0, 0, 0};

        public static final double turretCameraToTurretCenter = 0.072151;
    }

    public static class AutoAim {

        public static final double fieldLengthX = 16.54175;
        
        public static final Pose2d blueHubPose = new Pose2d(4.626, 4.035, new Rotation2d());
        public static final Pose2d redHubPose = new Pose2d(11.914, 4.035, new Rotation2d());

        public static final Pose2d bluePassShotHighPose = new Pose2d(3, 5.4, new Rotation2d());
        public static final Pose2d bluePassShotLowPose = new Pose2d(3, 2.6, new Rotation2d());

        public static final Pose2d redPassShotHighPose = new Pose2d(13.54175, 5.4, new Rotation2d());
        public static final Pose2d redPassShotLowPose = new Pose2d(13.54175, 2.6, new Rotation2d());

        public static final Pose2d outpostBlue = new Pose2d(0.861, 0.737, Rotation2d.kZero);
        public static final Pose2d outputRed = new Pose2d(15.769, 7.335, Rotation2d.kZero);
    }
}
