package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Autoaim extends SubsystemBase {

    private static final double DRIVEBASE_TILT_DEADBAND_RADIANS = 0.055;

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

    public record DrivebaseTiltDisplacement(Rotation2d turret, Rotation2d hood) {}

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
		this.passTimeOfFlightMap.put(3.0, PassTimeOfFlight.M3.getSeconds());
		this.passTimeOfFlightMap.put(5.0, PassTimeOfFlight.M5.getSeconds());
		this.passTimeOfFlightMap.put(7.0, PassTimeOfFlight.M7.getSeconds());
        this.passTimeOfFlightMap.put(9.0, PassTimeOfFlight.M9.getSeconds());
		this.passTimeOfFlightMap.put(11.0, PassTimeOfFlight.M11.getSeconds());
		this.passTimeOfFlightMap.put(13.0, PassTimeOfFlight.M13.getSeconds());

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

    public DrivebaseTiltDisplacement getDrivebaseTiltDisplacement() {
        double forwardTiltRadians = drivetrain.getPigeon2().getRoll().getValue().in(Radians);
        double sidewaysTiltRadians = drivetrain.getPigeon2().getPitch().getValue().in(Radians);
        return calculateDrivebaseTiltDisplacement(
            forwardTiltRadians,
            sidewaysTiltRadians,
            getTurretRelativeAngleToFireWhileMoving());
    }

    public static DrivebaseTiltDisplacement calculateDrivebaseTiltDisplacement(
            double forwardTiltRadians,
            double sidewaysTiltRadians,
            Rotation2d turretFacingAngle) {
        if (Math.hypot(forwardTiltRadians, sidewaysTiltRadians) < DRIVEBASE_TILT_DEADBAND_RADIANS) {
            return new DrivebaseTiltDisplacement(Rotation2d.kZero, Rotation2d.kZero);
        }

        double cos = turretFacingAngle.getCos();
        double sin = turretFacingAngle.getSin();

        double hoodDisplacementRadians = (forwardTiltRadians * cos) + (sidewaysTiltRadians * sin);
        double turretDisplacementRadians = (forwardTiltRadians * sin) - (sidewaysTiltRadians * cos);

        return new DrivebaseTiltDisplacement(
            new Rotation2d(turretDisplacementRadians),
            new Rotation2d(hoodDisplacementRadians));
    }

    public Rotation2d getTurretDisplacementFromDrivebaseTilt() {
        return getDrivebaseTiltDisplacement().turret();
    }

    public Rotation2d getHoodDisplacementFromDrivebaseTilt() {
        return getDrivebaseTiltDisplacement().hood();
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
        try {
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
        } catch (Exception ignored) {
        }
        
        try {
            Logger.recordOutput("Autoaim/target", new Pose2d(targetPose, new Rotation2d()));
            Logger.recordOutput("Autoaim/displacedTarget", new Pose2d(targetDisplacedIteratedPose, new Rotation2d()));
            Logger.recordOutput("Autoaim/iteratedTOF", iteratedTOF);
            Logger.recordOutput("Autoaim/targetAngle", this.getTurretRelativeAngleToFireWhileMoving());
            Logger.recordOutput("Autoaim/drivetrainAngle", drivetrain.getState().Pose.getRotation());
            Logger.recordOutput("Autoaim/distanceToTarget", this.getDistanceToTarget());
            Logger.recordOutput("Autoaim/distanceToDisplacedTarget", this.getDistanceToScoreWhileMoving());
            Logger.recordOutput("Autoaim/goodToPass", this.goodToPass());
            Logger.recordOutput("Autoaim/turretTiltDisplacement", this.getTurretDisplacementFromDrivebaseTilt());
            Logger.recordOutput("Autoaim/hoodTiltDisplacement", this.getHoodDisplacementFromDrivebaseTilt());
        } catch (Exception e) {
            System.out.println("Error logging autoaim values: " + e.getMessage());
        }
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
                    passCycleTOF = passTimeOfFlightMap.get(getDistanceToTargetWhileMoving(passCycleTOF));
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

		M1(0.2),
		M3(0.4),
		M5(0.6),
		M7(0.8),
		M9(1),
		M11(1.2),
		M13(1.4);

		private final double sec;

		PassTimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}
}
