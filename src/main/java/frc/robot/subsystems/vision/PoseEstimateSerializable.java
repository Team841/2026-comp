package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class PoseEstimateSerializable extends LimelightHelpers.PoseEstimate implements StructSerializable{

    public static LimelightHelpers.RawFiducial blankFiducial[] = new RawFiducial[0];

    public PoseEstimateSerializable() {
        this.pose = new Pose2d();
        this.timestampSeconds = 0;
        this.latency = 0;
        this.tagCount = 0;
        this.tagSpan = 0;
        this.avgTagDist = 0;
        this.avgTagArea = 0;
        this.rawFiducials = new RawFiducial[]{};
        this.isMegaTag2 = false;
    }

    public PoseEstimateSerializable(Pose2d pose, double timestampSeconds, double latency, 
        int tagCount, double tagSpan, double avgTagDist, 
        double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {

        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.latency = latency;
        this.tagCount = tagCount;
        this.tagSpan = tagSpan;
        this.avgTagDist = avgTagDist; 
        this.avgTagArea = avgTagArea;
        this.rawFiducials = rawFiducials;
        this.isMegaTag2 = isMegaTag2;
    }

    public static final PoseEstimateSerializableStruct poseEstimateStruct = new PoseEstimateSerializableStruct();

    public static class PoseEstimateSerializableStruct implements Struct<PoseEstimateSerializable> {

        @Override
        public Class<PoseEstimateSerializable> getTypeClass() {
            return PoseEstimateSerializable.class;
        }

        @Override
        public String getTypeName() {
            return "class:PoseEstimateSerializable";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + 3 * Double.BYTES + 2 * Integer.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d pose; double timestampSeconds; double latency; int tagCount; double tagSpan; double avgTagDist; double avgTagArea; int isMegaTag2;";
        }
        
        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public PoseEstimateSerializable unpack(ByteBuffer bb) {
            Pose2d pose = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            double latency = bb.getDouble();
            int tagCount = bb.getInt();
            double tagSpan = bb.getDouble();
            double avgTagDist = bb.getDouble();
            double avgTagArea = bb.getDouble();
            boolean isMegaTag2 = bb.getInt() == 1;
            return new PoseEstimateSerializable(
                pose,
                timestampSeconds,
                latency,
                tagCount,
                tagSpan,
                avgTagDist,
                avgTagArea,
                blankFiducial,
                isMegaTag2
            );
        }

        @Override
        public void pack(ByteBuffer bb, PoseEstimateSerializable value) {
            Pose2d.struct.pack(bb, value.pose);
            bb.putDouble(value.timestampSeconds);
            bb.putDouble(value.latency);
            bb.putInt(value.tagCount);
            bb.putDouble(value.tagSpan);
            bb.putDouble(value.avgTagDist);
            bb.putDouble(value.avgTagArea);
            bb.putInt(value.isMegaTag2 ? 1 : 0);
        }
    }
}
