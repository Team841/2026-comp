// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final SendableChooser<Command> autoChooser;

    private final RobotContainer robotContainer;
    private final Turret turret = new Turret();
    private final Drivetrain drivetrain;
    private final DyeRotor dyeRotor = new DyeRotor();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final IntakePivot intakePivot = new IntakePivot();
    private final Shooter shooter = new Shooter();
    private final LED led;

    public final VisionIO visionIO;
    public final Vision vision;

    private Orchestra orchestra = new Orchestra();

    public Robot() {
//        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

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

        this.visionIO = new VisionIOLimelights();
        this.vision = new Vision(visionIO, drivetrain, turret);

        robotContainer = new RobotContainer(drivetrain, dyeRotor, hood, intake, intakePivot, shooter, turret, visionIO, vision);

        if (!AutoBuilder.isConfigured()){
            drivetrain.ConfigureAutobuilder();
        }

        this.led = new LED(robotContainer);
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        var latestShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        Logger.recordOutput("HubShift/Official", latestShiftInfo);

        SmartDashboard.putString("Current Shift", latestShiftInfo.currentShift().toString());
//        SmartDashboard.putNumber("Elapsed Time", latestShiftInfo.elapsedTime());
        SmartDashboard.putNumber("Remaining Time", latestShiftInfo.remainingTime());
        SmartDashboard.putBoolean("Shift Active", latestShiftInfo.active());
    }

    @Override
    public void disabledInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected().finallyDo(() -> {
            RobotConstants.currentAimMode = RobotConstants.autoAimMode.HUB;
        });

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        HubShiftUtil.initialize();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        HubShiftUtil.initialize();
    }

    @Override
    public void teleopPeriodic() {}

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
