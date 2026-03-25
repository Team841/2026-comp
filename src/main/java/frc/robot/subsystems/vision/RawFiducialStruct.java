package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.LimelightHelpers;

import java.nio.ByteBuffer;
import java.util.Arrays;

public record RawFiducialStruct(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity)
        implements StructSerializable {

    public static final RawFiducialStruct[] blankArray = new RawFiducialStruct[0];

    public static RawFiducialStruct fromRawFiducial(LimelightHelpers.RawFiducial raw) {
        return new RawFiducialStruct(
                raw.id, raw.txnc, raw.tync, raw.ta, raw.distToCamera, raw.distToRobot, raw.ambiguity
        );
    }

    public static RawFiducialStruct[] fromRawFiducial(LimelightHelpers.RawFiducial[] raws) {
        if (raws.length == 0){
            return blankArray;
        }
        return Arrays.stream(raws)
                .map(RawFiducialStruct::fromRawFiducial)
                .toArray(RawFiducialStruct[]::new);
    }

    public static final Struct<RawFiducialStruct> struct =
            new Struct<RawFiducialStruct>() {
                @Override
                public Class<RawFiducialStruct> getTypeClass() {
                    return RawFiducialStruct.class;
                }

                @Override
                public String getTypeName() {
                    return "class:RawFiducialStruct";
                }

                @Override
                public int getSize() {
                    return Integer.BYTES + Double.BYTES * 6;
                }

                @Override
                public String getSchema() {
                    return "int id; double txnc; double tync; double ta; double distToCamera; double distToRobot; double ambiguity;";
                }

                @Override
                public RawFiducialStruct unpack(ByteBuffer bb) {
                    int id = bb.getInt();
                    double txnc = bb.getDouble();
                    double tync = bb.getDouble();
                    double ta = bb.getDouble();
                    double distToCamera = bb.getDouble();
                    double distToRobot = bb.getDouble();
                    double ambiguity = bb.getDouble();
                    return new RawFiducialStruct(
                            id, txnc, tync, ta, distToCamera, distToRobot, ambiguity
                    );
                }

                @Override
                public void pack(ByteBuffer bb, RawFiducialStruct value) {
                    bb.putDouble(value.id);
                    bb.putDouble(value.txnc);
                    bb.putDouble(value.tync);
                    bb.putDouble(value.ta);
                    bb.putDouble(value.distToCamera);
                    bb.putDouble(value.distToRobot);
                    bb.putDouble(value.ambiguity);
                }
            };
}
