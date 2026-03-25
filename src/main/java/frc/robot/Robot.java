// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelights;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

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

    public final VisionIO visionIO;
    public final Vision vision;

    public Robot() {
//        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
//        Logger.addDataReceiver(new RLOGServer());

        SignalLogger.enableAutoLogging(false);

        Logger.start();

        drivetrain = new Drivetrain(
        TunerConstants.DrivetrainConstants, 
        TunerConstants.FrontLeft, 
        TunerConstants.FrontRight, 
        TunerConstants.BackLeft, 
        TunerConstants.BackRight);

        this.visionIO = new VisionIOLimelights();
        this.vision = new Vision(visionIO, drivetrain);

        robotContainer = new RobotContainer(drivetrain, dyeRotor, hood, intake, intakePivot, shooter, turret, visionIO, vision);

        if (!AutoBuilder.isConfigured()){
            drivetrain.ConfigureAutobuilder();
        }
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Threads.setCurrentThreadPriority(true, 5);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
        Logger.recordOutput("RobotState/RobotMode", robotContainer.currentMode);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
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
