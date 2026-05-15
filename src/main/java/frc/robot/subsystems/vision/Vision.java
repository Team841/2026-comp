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
import frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Drivetrain drivetrain;
    private final Turret turret;

    public static final Vector<N3> standardVisionDevs2OrMore = VecBuilder.fill(0.3, 0.3, 0.3);
    public static final Vector<N3> standardVisionDevs1tag = VecBuilder.fill(0.5, 0.5, 0.5);

    public static final Vector<N3> standardVisionDevsMT2 = VecBuilder.fill(1, 1, 999);
    public static final Vector<N3> standardVisionDevsMT1 = VecBuilder.fill(999, 999, 1);

    public boolean disableVision = false;
    public boolean disableTurretCamera = false;

    public Vision(VisionIO io, Drivetrain drivetrain, Turret turret) {
        this.io = io;
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        SwerveDrivetrain.SwerveDriveState lastDriveState = this.drivetrain.getState();
        inputs.robotYawDegrees = lastDriveState.Pose.getRotation().getMeasure().in(Units.Degree);
        inputs.robotYawRateDegreesPerSecond = Math.toDegrees(lastDriveState.Speeds.omegaRadiansPerSecond);
        inputs.robotRollDegrees = this.drivetrain.getPigeon2().getRoll().getValue().in(Units.Degree);
        inputs.robotRollRateDegreesPerSecond = 0;
        inputs.robotPitchDegrees = this.drivetrain.getPigeon2().getPitch().getValue().in(Units.Degree);
        inputs.robotPitchRateDegreesPerSecond = 0;
        inputs.turretAngle = turret.getTurretAngleAbsolute();

        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        boolean reject = false;

        if (Math.abs(lastDriveState.Speeds.omegaRadiansPerSecond) > 3) {
            reject = true;
        }

        if (disableVision) {
            reject = true;
        }

        if (!reject) {
            try {
                if (inputs.frontLeftHasTarget) {
                    filterLL(
                        inputs.frontLeftPoseEstimateMT1.pose(), 
                        inputs.frontLeftPoseEstimateMT1.tagCount(), 
                        inputs.frontLeftPoseEstimateMT1.avgTagArea(), 
                        inputs.frontLeftPoseEstimateMT1.timestampSeconds(), 
                        false);
                    // if (!inputs.turretHasTarget) {
                        filterLL(
                        inputs.frontLeftPoseEstimateMT2.pose(), 
                        inputs.frontLeftPoseEstimateMT2.tagCount(), 
                        inputs.frontLeftPoseEstimateMT2.avgTagArea(), 
                        inputs.frontLeftPoseEstimateMT2.timestampSeconds(), 
                        true);
                    // }
                }

                // if (inputs.backLeftHasTarget) {
                    // filterLL(
                    //     inputs.backLeftPoseEstimateMT1.pose(), 
                    //     inputs.backLeftPoseEstimateMT1.tagCount(), 
                    //     inputs.backLeftPoseEstimateMT1.avgTagArea(), 
                    //     inputs.backLeftPoseEstimateMT1.timestampSeconds(), 
                    //     false);
                        
                    // if (!inputs.turretHasTarget) {
                        // filterLL(
                        //     inputs.backLeftPoseEstimateMT2.pose(), 
                        //     inputs.backLeftPoseEstimateMT2.tagCount(), 
                        //     inputs.backLeftPoseEstimateMT2.avgTagArea(), 
                        //     inputs.backLeftPoseEstimateMT2.timestampSeconds(), 
                        //     true);
                    // }
                // }

                // if (inputs.frontRightHasTarget) {
                    // filterLL(
                    //     inputs.frontRightPoseEstimateMT1.pose(), 
                    //     inputs.frontRightPoseEstimateMT1.tagCount(), 
                    //     inputs.frontRightPoseEstimateMT1.avgTagArea(), 
                    //     inputs.frontRightPoseEstimateMT1.timestampSeconds(), 
                    //     false);
                    // if (!inputs.turretHasTarget) {
                        // filterLL(
                        //     inputs.frontRightPoseEstimateMT2.pose(), 
                        //     inputs.frontRightPoseEstimateMT2.tagCount(), 
                        //     inputs.frontRightPoseEstimateMT2.avgTagArea(), 
                        //     inputs.frontRightPoseEstimateMT2.timestampSeconds(), 
                        //     true);
                    // }
                // }

                if (inputs.backRightHasTarget) {
                    filterLL(
                        inputs.backRightPoseEstimateMT1.pose(), 
                        inputs.backRightPoseEstimateMT1.tagCount(), 
                        inputs.backRightPoseEstimateMT1.avgTagArea(), 
                        inputs.backRightPoseEstimateMT1.timestampSeconds(), 
                        false);
                    // if (!inputs.turretHasTarget) {
                        filterLL(
                            inputs.backRightPoseEstimateMT2.pose(), 
                            inputs.backRightPoseEstimateMT2.tagCount(), 
                            inputs.backRightPoseEstimateMT2.avgTagArea(), 
                            inputs.backRightPoseEstimateMT2.timestampSeconds(), 
                            true);
                    // }
                }

                if (inputs.turretHasTarget && !inputs.backRightHasTarget && !inputs.frontLeftHasTarget && !disableTurretCamera && Math.abs(turret.getTurretVelocityInRadiansPerSecond()) < 5){
                    filterLL(
                        inputs.turretPoseEstimateMT2.pose(), 
                        inputs.turretPoseEstimateMT2.tagCount(), 
                        inputs.turretPoseEstimateMT2.avgTagArea(), 
                        inputs.turretPoseEstimateMT2.timestampSeconds(),
                        true);
                }

            } catch (Exception ignored) {
            }
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    private void filterLL(Pose2d pose, int tagCount, double avgTagArea, double timestampSeconds, boolean isMT2) {
        if (tagCount > 0) {
            if (tagCount == 1 && isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs1tag.elementTimes(standardVisionDevsMT2)
                );
            } else if (tagCount > 1 && isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.elementTimes(standardVisionDevsMT2)
                );
            } else if (tagCount > 2 && !isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.elementTimes(standardVisionDevsMT1)
                );
            }
        }
    }

    public void disableVision() {
        disableVision = true;
    }

    public void enableVision() {
        disableVision = false;
    }

    public void disableTurretVision() {
        disableTurretCamera = true;
    }

    public void enableTurretVision() {
        disableTurretCamera = false;
    }
}
