package frc.robot.constants;

import java.util.function.BooleanSupplier;

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
        public static final double[] turretPose = {0, 0, 0, 0, 0, 0};
    }
}