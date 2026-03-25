package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Drivetrain drivetrain;

    public static final Vector<N3> standardVisionDevs2OrMore = VecBuilder.fill(0.3, 0.3, 0.3);
    public static final Vector<N3> standardVisionDevs1tag = VecBuilder.fill(0.75, 0.75, 0.75);


    public Vision(VisionIO io, Drivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        SwerveDrivetrain.SwerveDriveState lastDriveState = this.drivetrain.getState();
        inputs.robotYawDegrees = lastDriveState.Pose.getRotation().getDegrees();
        inputs.robotYawRateDegreesPerSecond = lastDriveState.Speeds.omegaRadiansPerSecond;
        inputs.robotRollDegrees = this.drivetrain.getPigeon2().getRoll().getValue().in(Units.Degree);
        inputs.robotRollRateDegreesPerSecond = 0;
        inputs.robotPitchDegrees = this.drivetrain.getPigeon2().getPitch().getValue().in(Units.Degree);
        inputs.robotPitchRateDegreesPerSecond = 0;

        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        boolean reject = false;

        if (Math.abs(lastDriveState.Speeds.omegaRadiansPerSecond) > 1.5){
            reject = true;
        }

        if (!reject){
            try{
                if (inputs.leftHasTarget){
                    filterLL(inputs.leftPoseEstimateMT1.pose(), inputs.leftPoseEstimateMT1.tagCount(), inputs.leftPoseEstimateMT1.avgTagArea(), inputs.leftPoseEstimateMT1.timestampSeconds());
                    filterLL(inputs.leftPoseEstimateMT2.pose(), inputs.leftPoseEstimateMT2.tagCount(), inputs.leftPoseEstimateMT2.avgTagArea(), inputs.leftPoseEstimateMT2.timestampSeconds());
                }

                if (inputs.rightHasTarget) {
                    filterLL(inputs.rightPoseEstimateMT1.pose(), inputs.rightPoseEstimateMT1.tagCount(), inputs.rightPoseEstimateMT1.avgTagArea(), inputs.rightPoseEstimateMT1.timestampSeconds());
                    filterLL(inputs.rightPoseEstimateMT2.pose(), inputs.rightPoseEstimateMT2.tagCount(), inputs.rightPoseEstimateMT2.avgTagArea(), inputs.rightPoseEstimateMT2.timestampSeconds());
                }
            } catch (Exception ignored) { }
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    private void filterLL(Pose2d pose, int tagCount, double avgTagArea, double timestampSeconds){
        if (tagCount > 0){
            if (tagCount == 1){
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs1tag.times(avgTagArea)
                );

                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs1tag.times(avgTagArea)
                );
            }
            else {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.times(avgTagArea)
                );

                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.times(avgTagArea)
                );
            }
        }
    }
}
