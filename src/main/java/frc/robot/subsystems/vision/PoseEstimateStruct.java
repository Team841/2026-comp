package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.LimelightHelpers;

import java.nio.ByteBuffer;

public final class PoseEstimateStruct implements StructSerializable {

    private final Pose2d pose;
    private final double timestampSeconds;
    private final double latency;
    private final int tagCount;
    private final double tagSpan;
    private final double avgTagDist;
    private final double avgTagArea;
    private final int isMegaTag2;

    public PoseEstimateStruct(
            Pose2d pose,
            double timestampSeconds,
            double latency,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            int isMegaTag2
    ) {
        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.latency = latency;
        this.tagCount = tagCount;
        this.tagSpan = tagSpan;
        this.avgTagDist = avgTagDist;
        this.avgTagArea = avgTagArea;
        this.isMegaTag2 = isMegaTag2;
    }

    public Pose2d pose() {
        return pose;
    }

    public double timestampSeconds() {
        return timestampSeconds;
    }

    public double latency() {
        return latency;
    }

    public int tagCount() {
        return tagCount;
    }

    public double tagSpan() {
        return tagSpan;
    }

    public double avgTagDist() {
        return avgTagDist;
    }

    public double avgTagArea() {
        return avgTagArea;
    }

    public int isMegaTag2() {
        return isMegaTag2;
    }

    public static PoseEstimateStruct fromPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
        return new PoseEstimateStruct(
                estimate.pose, estimate.timestampSeconds, estimate.latency, estimate.tagCount, estimate.tagSpan, estimate.avgTagDist, estimate.avgTagArea, estimate.isMegaTag2 ? 1 : 0
        );
    }

    public static final LimelightHelpers.RawFiducial[] blankFiducial = new LimelightHelpers.RawFiducial[0];

    public static final Struct<PoseEstimateStruct> struct =
            new Struct<PoseEstimateStruct>() {
                @Override
                public Class<PoseEstimateStruct> getTypeClass() {
                    return PoseEstimateStruct.class;
                }

                @Override
                public String getTypeName() {
                    return "PoseEstimateStruct";
                }

                @Override
                public int getSize() {
                    return Pose2d.struct.getSize() + 5 * Double.BYTES + 2 * Integer.BYTES;
                }

                @Override
                public String getSchema() {
                    return "Pose2d pose;double timestampSeconds;double latency;int tagCount;double tagSpan;double avgTagDist;double avgTagArea;int isMegaTag2";
                }

                @Override
                public Struct<?>[] getNested() {
                    return new Struct<?>[] {Pose2d.struct};
                }


                @Override
                public PoseEstimateStruct unpack(ByteBuffer bb) {
                    Pose2d pose = Pose2d.struct.unpack(bb);
                    double timestampSeconds = bb.getDouble();
                    double latency = bb.getDouble();
                    int tagCount = bb.getInt();
                    double tagSpan = bb.getDouble();
                    double avgTagDist = bb.getDouble();
                    double avgTagArea = bb.getDouble();
                    int isMegaTag2 = bb.getInt();
                    return new PoseEstimateStruct(
                            pose, timestampSeconds, latency, tagCount, tagSpan, avgTagDist, avgTagArea, isMegaTag2
                    );
                }

                @Override
                public void pack(ByteBuffer bb, PoseEstimateStruct value) {
                    Pose2d.struct.pack(bb, value.pose);
                    bb.putDouble(value.timestampSeconds);
                    bb.putDouble(value.latency);
                    bb.putInt(value.tagCount);
                    bb.putDouble(value.tagSpan);
                    bb.putDouble(value.avgTagDist);
                    bb.putDouble(value.avgTagArea);
                    bb.putInt(value.isMegaTag2);
                }
            };
}
