package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.LimelightHelpers;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SuperstructureConstants;

import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class TurretIOReal extends TurretIO {

    private final double turretCameraRadiusToCenter = 0.072151;

    TurretIOInputs inputCache = new TurretIOInputs();

    private TalonFX turretMotor;
    private TalonFX hoodMotor;
    private MotionMagicExpoVoltage turretPositionControl = new MotionMagicExpoVoltage(0);
    private MotionMagicExpoVoltage hoodPositionControl = new MotionMagicExpoVoltage(0);

    public TurretThread turretThread;
    protected final ReadWriteLock inwardLock = new ReentrantReadWriteLock();
    protected final ReadWriteLock outwardLock = new ReentrantReadWriteLock();

    protected Rotation2d targetAngle = new Rotation2d();
    protected Rotation2d currentAngle = new Rotation2d();
    protected double[] cameraPose = RobotConstants.Vision.turretPose;
    protected Pose2d latestCameraPose = new Pose2d();
    protected AtomicReference<TurretModes> turretMode = new AtomicReference<>(TurretModes.AUTOAIM_FIRE);

    public TurretIOReal(){
        this.turretThread = new TurretThread();

        this.turretMotor = new TalonFX(SuperstructureConstants.IDs.turretMotorID, "rio");
        this.turretMotor.getConfigurator().apply(SuperstructureConstants.TurretConstants.turretMotorConfigs);

        this.hoodMotor = new TalonFX(SuperstructureConstants.IDs.hoodMotorID, "rio");
        this.hoodMotor.getConfigurator().apply(SuperstructureConstants.HoodConstants.hoodMotorConfigs);
        this.zeroHood();
    }

    public double avgWrappedAngle(double a1, double a2){
        double diff = a1 - a2;
        while (diff > Math.PI)
            diff -= 2 * Math.PI;
        while (diff < -Math.PI)
            diff += 2 * Math.PI;
        double avg_theta = a1 + 0.5 * diff;
        return avg_theta % (2 * Math.PI);
    }

    public void zeroHood(){
        this.hoodMotor.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        this.inputCache = inputs;
        setLLSettings();

        inputs.turretCameraHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.turretName);

        if (inputs.turretCameraHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.turretName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.turretName);
            inputs.poseEstimateMT1 = mt1;
            inputs.poseEstimateMT2 = mt2;
            inputs.rawFiducials = mt1.rawFiducials();

            inwardLock.writeLock().lock();
            try {
                Translation2d ax = mt1.pose().getTranslation().plus(mt2.pose().getTranslation()).div(2);
                latestCameraPose = new Pose2d(
                        ax,
                        new Rotation2d(avgWrappedAngle(mt1.pose().getRotation().getRadians(), mt2.pose().getRotation().getRadians()))
                );
            } finally {
                inwardLock.writeLock().unlock();
            }
        }
    }

    private void setLLSettings(){
        outwardLock.readLock().lock();
        try {
            LimelightHelpers.setLimelightNTDoubleArray(RobotConstants.Vision.turretName, "camerapose_robotspace_set", cameraPose);
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.turretName, 3);
        } finally {
            outwardLock.readLock().unlock();
        }
    }

    public class TurretThread{
        protected final Thread thread;
        protected volatile boolean running = false;
        private final long INTERVAL_NANOS = 1_000_000_000L / 250;
        public TurretThread(){
            this.thread = new Thread(this::run);
            this.thread.setDaemon(true);
        }

        public void start(){
            this.running = true;
            this.thread.start();;
        }

        public void stop(){
            this.running = false;
            try{
                this.thread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        private void run() {
            Threads.setCurrentThreadPriority(true, 4);
            long nextTick = System.nanoTime();
            while (this.running){

                outwardLock.writeLock().lock();
                try {
                    currentAngle = new Rotation2d(-turretMotor.getPosition().getValueAsDouble() * (Math.PI / 6.25));
                    cameraPose[0] = turretCameraRadiusToCenter * currentAngle.getCos();
                    cameraPose[1] = turretCameraRadiusToCenter * currentAngle.getSin();
                    cameraPose[5] = currentAngle.getDegrees();
                } finally {
                    outwardLock.writeLock().unlock();
                }

                inwardLock.readLock().lock();
                try {
                    continue;
                } finally {
                    inwardLock.readLock().unlock();
                }


                long delay = System.nanoTime() - nextTick;
                if (delay >= 0) {
                    LockSupport.parkNanos(delay);
                } else {
                    nextTick = System.nanoTime();
                }
            }
        }
    }

}
