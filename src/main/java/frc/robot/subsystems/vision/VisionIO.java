package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
    class VisionIOInputs implements LoggableInputs {
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

        public Rotation2d turretAngle = new Rotation2d();

        @Override
        public void toLog(LogTable table) {
            table.put("turretHasTarget", turretHasTarget);
            table.put("frontLeftHasTarget", frontLeftHasTarget);
            table.put("frontRightHasTarget", frontRightHasTarget);
            table.put("backLeftHasTarget", backLeftHasTarget);
            table.put("backRightHasTarget", backRightHasTarget);

            table.put("frontLeftRawFiducial", rawFiducialsToLogArray(frontLeftRawFiducial));
            table.put("frontRightRawFiducial", rawFiducialsToLogArray(frontRightRawFiducial));
            table.put("backLeftRawFiducial", rawFiducialsToLogArray(backLeftRawFiducial));
            table.put("backRightRawFiducial", rawFiducialsToLogArray(backRightRawFiducial));
            table.put("turretRawFiducial", rawFiducialsToLogArray(turretRawFiducial));

            logPoseEstimate(table, "frontLeftPoseEstimateMT1", frontLeftPoseEstimateMT1);
            logPoseEstimate(table, "frontLeftPoseEstimateMT2", frontLeftPoseEstimateMT2);
            logPoseEstimate(table, "frontRightPoseEstimateMT1", frontRightPoseEstimateMT1);
            logPoseEstimate(table, "frontRightPoseEstimateMT2", frontRightPoseEstimateMT2);
            logPoseEstimate(table, "backLeftPoseEstimateMT1", backLeftPoseEstimateMT1);
            logPoseEstimate(table, "backLeftPoseEstimateMT2", backLeftPoseEstimateMT2);
            logPoseEstimate(table, "backRightPoseEstimateMT1", backRightPoseEstimateMT1);
            logPoseEstimate(table, "backRightPoseEstimateMT2", backRightPoseEstimateMT2);
            logPoseEstimate(table, "turretPoseEstimateMT1", turretPoseEstimateMT1);
            logPoseEstimate(table, "turretPoseEstimateMT2", turretPoseEstimateMT2);

            table.put("robotYawDegrees", robotYawDegrees);
            table.put("robotYawRateDegreesPerSecond", robotYawRateDegreesPerSecond);
            table.put("robotPitchDegrees", robotPitchDegrees);
            table.put("robotPitchRateDegreesPerSecond", robotPitchRateDegreesPerSecond);
            table.put("robotRollDegrees", robotRollDegrees);
            table.put("robotRollRateDegreesPerSecond", robotRollRateDegreesPerSecond);
            table.put("turretAngle", turretAngle);
        }

        @Override
        public void fromLog(LogTable table) {
            turretHasTarget = table.get("turretHasTarget", turretHasTarget);
            frontLeftHasTarget = table.get("frontLeftHasTarget", frontLeftHasTarget);
            frontRightHasTarget = table.get("frontRightHasTarget", frontRightHasTarget);
            backLeftHasTarget = table.get("backLeftHasTarget", backLeftHasTarget);
            backRightHasTarget = table.get("backRightHasTarget", backRightHasTarget);

            frontLeftRawFiducial = rawFiducialsFromLogArray(table.get("frontLeftRawFiducial", rawFiducialsToLogArray(frontLeftRawFiducial)));
            frontRightRawFiducial = rawFiducialsFromLogArray(table.get("frontRightRawFiducial", rawFiducialsToLogArray(frontRightRawFiducial)));
            backLeftRawFiducial = rawFiducialsFromLogArray(table.get("backLeftRawFiducial", rawFiducialsToLogArray(backLeftRawFiducial)));
            backRightRawFiducial = rawFiducialsFromLogArray(table.get("backRightRawFiducial", rawFiducialsToLogArray(backRightRawFiducial)));
            turretRawFiducial = rawFiducialsFromLogArray(table.get("turretRawFiducial", rawFiducialsToLogArray(turretRawFiducial)));

            frontLeftPoseEstimateMT1 = readPoseEstimate(table, "frontLeftPoseEstimateMT1", frontLeftPoseEstimateMT1);
            frontLeftPoseEstimateMT2 = readPoseEstimate(table, "frontLeftPoseEstimateMT2", frontLeftPoseEstimateMT2);
            frontRightPoseEstimateMT1 = readPoseEstimate(table, "frontRightPoseEstimateMT1", frontRightPoseEstimateMT1);
            frontRightPoseEstimateMT2 = readPoseEstimate(table, "frontRightPoseEstimateMT2", frontRightPoseEstimateMT2);
            backLeftPoseEstimateMT1 = readPoseEstimate(table, "backLeftPoseEstimateMT1", backLeftPoseEstimateMT1);
            backLeftPoseEstimateMT2 = readPoseEstimate(table, "backLeftPoseEstimateMT2", backLeftPoseEstimateMT2);
            backRightPoseEstimateMT1 = readPoseEstimate(table, "backRightPoseEstimateMT1", backRightPoseEstimateMT1);
            backRightPoseEstimateMT2 = readPoseEstimate(table, "backRightPoseEstimateMT2", backRightPoseEstimateMT2);
            turretPoseEstimateMT1 = readPoseEstimate(table, "turretPoseEstimateMT1", turretPoseEstimateMT1);
            turretPoseEstimateMT2 = readPoseEstimate(table, "turretPoseEstimateMT2", turretPoseEstimateMT2);

            robotYawDegrees = table.get("robotYawDegrees", robotYawDegrees);
            robotYawRateDegreesPerSecond = table.get("robotYawRateDegreesPerSecond", robotYawRateDegreesPerSecond);
            robotPitchDegrees = table.get("robotPitchDegrees", robotPitchDegrees);
            robotPitchRateDegreesPerSecond = table.get("robotPitchRateDegreesPerSecond", robotPitchRateDegreesPerSecond);
            robotRollDegrees = table.get("robotRollDegrees", robotRollDegrees);
            robotRollRateDegreesPerSecond = table.get("robotRollRateDegreesPerSecond", robotRollRateDegreesPerSecond);
            turretAngle = table.get("turretAngle", turretAngle);
        }

        private static void logPoseEstimate(LogTable table, String key, LimelightHelpers.PoseEstimate estimate) {
            LogTable poseTable = table.getSubtable(key);
            poseTable.put("pose", estimate.pose());
            poseTable.put("timestampSeconds", estimate.timestampSeconds());
            poseTable.put("latency", estimate.latency());
            poseTable.put("tagCount", estimate.tagCount());
            poseTable.put("tagSpan", estimate.tagSpan());
            poseTable.put("avgTagDist", estimate.avgTagDist());
            poseTable.put("avgTagArea", estimate.avgTagArea());
            poseTable.put("rawFiducials", rawFiducialsToLogArray(estimate.rawFiducials()));
            poseTable.put("isMegaTag2", estimate.isMegaTag2());
        }

        private static LimelightHelpers.PoseEstimate readPoseEstimate(
                LogTable table, String key, LimelightHelpers.PoseEstimate defaultValue) {
            LogTable poseTable = table.getSubtable(key);
            return new LimelightHelpers.PoseEstimate(
                    poseTable.get("pose", defaultValue.pose()),
                    poseTable.get("timestampSeconds", defaultValue.timestampSeconds()),
                    poseTable.get("latency", defaultValue.latency()),
                    poseTable.get("tagCount", defaultValue.tagCount()),
                    poseTable.get("tagSpan", defaultValue.tagSpan()),
                    poseTable.get("avgTagDist", defaultValue.avgTagDist()),
                    poseTable.get("avgTagArea", defaultValue.avgTagArea()),
                    rawFiducialsFromLogArray(poseTable.get("rawFiducials", rawFiducialsToLogArray(defaultValue.rawFiducials()))),
                    poseTable.get("isMegaTag2", defaultValue.isMegaTag2()));
        }

        private static double[][] rawFiducialsToLogArray(LimelightHelpers.RawFiducial[] rawFiducials) {
            double[][] values = new double[rawFiducials.length][7];
            for (int i = 0; i < rawFiducials.length; i++) {
                values[i][0] = rawFiducials[i].id();
                values[i][1] = rawFiducials[i].txnc();
                values[i][2] = rawFiducials[i].tync();
                values[i][3] = rawFiducials[i].ta();
                values[i][4] = rawFiducials[i].distToCamera();
                values[i][5] = rawFiducials[i].distToRobot();
                values[i][6] = rawFiducials[i].ambiguity();
            }
            return values;
        }

        private static LimelightHelpers.RawFiducial[] rawFiducialsFromLogArray(double[][] values) {
            LimelightHelpers.RawFiducial[] rawFiducials = new LimelightHelpers.RawFiducial[values.length];
            for (int i = 0; i < values.length; i++) {
                double[] value = values[i];
                rawFiducials[i] = new LimelightHelpers.RawFiducial(
                        value.length > 0 ? (int) value[0] : 0,
                        value.length > 1 ? value[1] : 0.0,
                        value.length > 2 ? value[2] : 0.0,
                        value.length > 3 ? value[3] : 0.0,
                        value.length > 4 ? value[4] : 0.0,
                        value.length > 5 ? value[5] : 0.0,
                        value.length > 6 ? value[6] : 0.0);
            }
            return rawFiducials;
        }
    }

    void updateInputs(VisionIOInputs inputs);
}
