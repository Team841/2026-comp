// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");

    DoublePublisher turretPosition =
    table.getDoubleTopic("TurretPosition").publish();

    DoublePublisher turretTargetPosition =
    table.getDoubleTopic("TurretTargetPosition").publish();
    
    BooleanPublisher turretAtPosition =
    table.getBooleanTopic("TurretAtPosition").publish();

    DoublePublisher shooterVelocity =
    table.getDoubleTopic("ShooterVelocity").publish();

    DoublePublisher shooterTargetVelocity =
    table.getDoubleTopic("ShooterTargetVelocity").publish();
    
    BooleanPublisher ShooterAtSpeed =
    table.getBooleanTopic("ShooterAtSpeed").publish();

    private final RobotContainer robotContainer;
    private final Turret turret = new Turret();
    private final Drivetrain drivetrain;
    private final DyeRotor dyeRotor = new DyeRotor();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final IntakePivot intakePivot = new IntakePivot();
    private final Shooter shooter = new Shooter();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        SignalLogger.enableAutoLogging(false);

        Logger.start();

        drivetrain = new Drivetrain(
        () -> turret.getPositionInRadians_ZerotoTwoPi(),
        () -> turret.isAtPosition(),
        TunerConstants.DrivetrainConstants, 
        TunerConstants.FrontLeft, 
        TunerConstants.FrontRight, 
        TunerConstants.BackLeft, 
        TunerConstants.BackRight);

        robotContainer = new RobotContainer(drivetrain, dyeRotor, hood, intake, intakePivot, shooter, turret);
    }

    @Override
    public void robotPeriodic() {
        // m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        turretPosition.set(this.turret.getPosition());
        turretTargetPosition.set(this.turret.getTurretTargetPosition());
        turretAtPosition.set(this.turret.isAtPosition());

        shooterVelocity.set(this.shooter.getShooterVelocity());
        shooterTargetVelocity.set(this.shooter.getShooterTargetVelocity());
        ShooterAtSpeed.set(this.shooter.atfullSpeed());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();

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
