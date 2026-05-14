// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer.RobotMode;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelights;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Autoaim.FiringLocation;
import frc.robot.subsystems.DyeRotor.RotorState;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Turret.TurretState;

public class Robot extends LoggedRobot {

    private final AutoChooser autoChooser;

    private final RobotContainer robotContainer;
    private final Autoaim autoaim;
    private final Turret turret;
    private final Drivetrain drivetrain;
    private final DyeRotor dyeRotor;
    private final Hood hood;
    private final Intake intake;
    private final IntakePivot intakePivot;
    private final Shooter shooter;
    private final LED led;

    public final VisionIO visionIO;
    public final Vision vision;

    private final AutoFactory autoFactory;

    private final Autos autos;

    private Orchestra orchestra = new Orchestra();

    public Robot() {
        // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //  Logger.addDataReceiver(new RLOGServer());

        DriverStation.silenceJoystickConnectionWarning(true);

        SignalLogger.enableAutoLogging(false);

        Logger.start();

        drivetrain = new Drivetrain(
            TunerConstants.DrivetrainConstants, 
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight);

        this.autoaim = new Autoaim(drivetrain);

        this.turret = new Turret(autoaim);
        this.dyeRotor = new DyeRotor();
        this.hood = new Hood(autoaim);
        this.intake = new Intake();
        this.intakePivot = new IntakePivot();
        this.shooter = new Shooter(autoaim);

        this.visionIO = new VisionIOLimelights();
        this.vision = new Vision(visionIO, drivetrain, turret);

        robotContainer = new RobotContainer(drivetrain, dyeRotor, hood, intake, intakePivot, shooter, turret, visionIO, vision, autoaim);

        this.led = new LED(robotContainer);

        this.autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );

        this.autos = new Autos(robotContainer, autoFactory);
        
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("L_NZ_DP_NZR", autos::LeftSideOneSweepPlusDepotAndReturn);
        autoChooser.addRoutine("R_NZ", autos::RightSideOneSweepNZ);
        autoChooser.addRoutine("L_NZ", autos::LeftSideOneSweepNZ);
        autoChooser.addRoutine("L_NZ_NZ", autos::LeftSideTwoSweep);
        autoChooser.addRoutine("R_NZ_NZ", autos::RightSideTwoSweep);
        autoChooser.addRoutine("R_NZ_OP_NZR", autos::RightSideOneSweepOutpostAndReturn);
        autoChooser.addRoutine("M_Preload", autos::MiddlePreloadFire);
        autoChooser.addRoutine("M_DP", autos::MiddleDepot);
        autoChooser.addRoutine("M_OP", autos::MiddleOutpost);

        SmartDashboard.putData("AutoChooser", autoChooser);

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        Threads.setCurrentThreadPriority(true, 5);

        // this.orchestra.addInstrument(this.dyeRotor.rotorMotor);
        // this.orchestra.addInstrument(this.intake.intakeMotorLeft);
        // this.orchestra.addInstrument(this.intake.intakeMotorRight);
        // this.orchestra.addInstrument(this.hood.hoodMotor);
        // this.orchestra.addInstrument(this.intakePivot.intakePivotMotor);
        // this.orchestra.addInstrument(this.shooter.rightMotor);
        // this.orchestra.addInstrument(this.shooter.leftMotor);
        // this.orchestra.addInstrument(this.turret.turretMotor);

        // this.orchestra.loadMusic("rickroll.chrp");

        // this.orchestra.play();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
        Logger.recordOutput("RobotState/RobotMode", robotContainer.currentMode);
        Logger.recordOutput("Drivetrain/TurretPose", new Pose2d(drivetrain.getState().Pose.getTranslation(), turret.getTurretAngleAbsolute().plus(drivetrain.getState().Pose.getRotation())));
        var latestShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        Logger.recordOutput("HubShift/Official", latestShiftInfo);

        SmartDashboard.putString("Current Shift", latestShiftInfo.currentShift().toString());
        // SmartDashboard.putNumber("Elapsed Time", latestShiftInfo.elapsedTime());
        SmartDashboard.putNumber("Remaining Time", latestShiftInfo.remainingTime());
        SmartDashboard.putBoolean("Shift Active", latestShiftInfo.active());
    }

    @Override
    public void disabledInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetThrottle(RobotConstants.Vision.backLeftName, 150);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.backRightName, 150);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontLeftName, 150);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontRightName, 150);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.turretName, 150);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        HubShiftUtil.initialize();
        vision.enableVision();
        vision.enableTurretVision();

        LimelightHelpers.SetThrottle(RobotConstants.Vision.backLeftName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.backRightName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontLeftName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontRightName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.turretName, 0);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        intake.setState(IntakeState.STOP);
        dyeRotor.setState(RotorState.STOP);
        turret.setState(TurretState.HOLD);
        hood.setState(HoodState.HOLD);
        autoaim.setFiringLocation(FiringLocation.HUB);
        robotContainer.setMode(RobotMode.NEUTRAL);
        HubShiftUtil.initialize();
        vision.enableVision();
        vision.enableTurretVision();

        LimelightHelpers.SetThrottle(RobotConstants.Vision.backLeftName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.backRightName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontLeftName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.frontRightName, 0);
        LimelightHelpers.SetThrottle(RobotConstants.Vision.turretName, 0);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
