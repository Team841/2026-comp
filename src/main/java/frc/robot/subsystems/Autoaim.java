package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Autoaim extends SubsystemBase {

    public double iteratedTOF;

    public Translation2d targetPose = new Translation2d();
    public Translation2d targetDisplacedIteratedPose = new Translation2d();

    public FiringLocation target = FiringLocation.HUB;

    private final Drivetrain drivetrain;

    private InterpolatingDoubleTreeMap hubTimeOfFlightMap;
    private InterpolatingDoubleTreeMap passTimeOfFlightMap;

    public enum FiringLocation {
        HUB,
        PASS
    }

    public Autoaim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.hubTimeOfFlightMap.put(1.0, HubTimeOfFlight.M1.getSeconds());
		this.hubTimeOfFlightMap.put(2.0, HubTimeOfFlight.M2.getSeconds());
		this.hubTimeOfFlightMap.put(3.0, HubTimeOfFlight.M3.getSeconds());
		this.hubTimeOfFlightMap.put(4.0, HubTimeOfFlight.M4.getSeconds());
		this.hubTimeOfFlightMap.put(5.0, HubTimeOfFlight.M5.getSeconds());
		this.hubTimeOfFlightMap.put(6.0, HubTimeOfFlight.M6.getSeconds());
		this.hubTimeOfFlightMap.put(7.0, HubTimeOfFlight.M7.getSeconds());

        this.passTimeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.passTimeOfFlightMap.put(1.0, PassTimeOfFlight.M1.getSeconds());
		this.passTimeOfFlightMap.put(2.0, PassTimeOfFlight.M2.getSeconds());
		this.passTimeOfFlightMap.put(3.0, PassTimeOfFlight.M3.getSeconds());
		this.passTimeOfFlightMap.put(4.0, PassTimeOfFlight.M4.getSeconds());
		this.passTimeOfFlightMap.put(5.0, PassTimeOfFlight.M5.getSeconds());
		this.passTimeOfFlightMap.put(6.0, PassTimeOfFlight.M6.getSeconds());
		this.passTimeOfFlightMap.put(7.0, PassTimeOfFlight.M7.getSeconds());
    }

    public void setFiringLocation(FiringLocation newLocation) {
        target = newLocation;
    }

    public Rotation2d getFieldRelativeAngleToFireWhileMoving() {
        return new Rotation2d(Math.atan2(targetDisplacedIteratedPose.getY() - drivetrain.getState().Pose.getY(), targetDisplacedIteratedPose.getX() - drivetrain.getState().Pose.getX()));
    }

    public Rotation2d getTurretRelativeAngleToFireWhileMoving() {
        return getFieldRelativeAngleToFireWhileMoving().minus(drivetrain.getState().Pose.getRotation());
    }

    public Rotation2d getTurretLeadAngleFromDrivetrainRotation() {
        return new Rotation2d(drivetrain.getState().Speeds.omegaRadiansPerSecond * 0.05);
    }

    public double getDistanceToScoreWhileMoving() {
        return drivetrain.getState().Pose.getTranslation().getDistance(targetDisplacedIteratedPose);
    }

    public boolean goodToPass() {
        return drivetrain.getState().Pose.getY() < 3.3 || drivetrain.getState().Pose.getY() > 4.7;
    }


    @Override
    public void periodic() {
        switch (target) {
            case HUB:
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    targetPose = RobotConstants.AutoAimConstants.redHubPose.getTranslation();
                } else {
                    targetPose = RobotConstants.AutoAimConstants.blueHubPose.getTranslation();
                }
                break;

            case PASS:
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    if (drivetrain.getState().Pose.getY() > 4.027) {
                        targetPose = RobotConstants.AutoAimConstants.redPassShotHighPose.getTranslation();
                    } else {
                        targetPose = RobotConstants.AutoAimConstants.redPassShotLowPose.getTranslation();
                    }
                } else {
                    if (drivetrain.getState().Pose.getY() > 4.027) {
                        targetPose = RobotConstants.AutoAimConstants.bluePassShotHighPose.getTranslation();
                    } else {
                        targetPose = RobotConstants.AutoAimConstants.bluePassShotLowPose.getTranslation();
                    }
                }

            default:
                break;
        }

        periodicTOF();
        
        Logger.recordOutput("Autoaim/target", new Pose2d(targetPose, new Rotation2d()));
        Logger.recordOutput("Autoaim/displacedTarget", new Pose2d(targetDisplacedIteratedPose, new Rotation2d()));
        Logger.recordOutput("Autoaim/iteratedTOF", iteratedTOF);
        Logger.recordOutput("Autoaim/targetAngle", this.getTurretRelativeAngleToFireWhileMoving());
        Logger.recordOutput("Autoaim/drivetrainAngle", drivetrain.getState().Pose.getRotation());
        Logger.recordOutput("Autoaim/distanceToTarget", this.getDistanceToTarget());
        Logger.recordOutput("Autoaim/distanceToDisplacedTarget", this.getDistanceToScoreWhileMoving());
        Logger.recordOutput("Autoaim/goodToPass", this.goodToPass());
    }

    public void periodicTOF() {
        switch (target) {
            case HUB:
                double hubCycleTOF = hubTimeOfFlightMap.get(getDistanceToTarget());
                for (int i = 0; i < 10; i++) {
                    hubCycleTOF = hubTimeOfFlightMap.get(getDistanceToTargetWhileMoving(hubCycleTOF));
                }
                iteratedTOF = hubCycleTOF;
                targetDisplacedIteratedPose = getDisplacedTarget(iteratedTOF);
                break;

            case PASS:
                double passCycleTOF = hubTimeOfFlightMap.get(getDistanceToTarget());
                for (int i = 0; i < 10; i++) {
                    passCycleTOF = hubTimeOfFlightMap.get(getDistanceToTargetWhileMoving(passCycleTOF));
                }
                iteratedTOF = passCycleTOF;
                targetDisplacedIteratedPose = getDisplacedTarget(iteratedTOF);
                break;
        
            default:
                break;
        }
    }

    private double getDistanceToTargetWhileMoving(double ballTOF) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
        Translation2d velocityDisplacementOverTime = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond * ballTOF, fieldRelativeSpeeds.vyMetersPerSecond * ballTOF);
        return targetPose.minus(velocityDisplacementOverTime).getDistance(drivetrain.getState().Pose.getTranslation());
    }

    private Translation2d getDisplacedTarget(double ballTOF) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
        Translation2d velocityDisplacementOverTime = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond * ballTOF, fieldRelativeSpeeds.vyMetersPerSecond * ballTOF);
        return targetPose.minus(velocityDisplacementOverTime);
    }

    private double getDistanceToTarget() {
        return targetPose.getDistance(drivetrain.getState().Pose.getTranslation());
    }

    private enum HubTimeOfFlight {

		M1(0.7),
		M2(0.9),
		M3(1.1),
		M4(1.4),
		M5(1.6),
		M6(1.8),
		M7(2);

		private final double sec;

		HubTimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}

    private enum PassTimeOfFlight {

		M1(0.7),
		M2(0.9),
		M3(1.1),
		M4(1.4),
		M5(1.6),
		M6(1.8),
		M7(2);

		private final double sec;

		PassTimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}
}
