// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController cojoystick = new CommandXboxController(1);

    public final Drivetrain drivetrain;
    public final DyeRotor dyeRotor;
    public final Hood hood;
    public final Intake intake;
    public final IntakePivot intakePivot;
    public final Shooter shooter;
    public final Turret turret;

    public RobotContainer(Drivetrain drivetrain, DyeRotor dyeRotor, Hood hood, Intake intake, IntakePivot intakePivot, Shooter shooter, Turret turret) {
        this.drivetrain = drivetrain;
        this.dyeRotor = dyeRotor;
        this.hood = hood;
        this.intake = intake;
        this.intakePivot = intakePivot;
        this.shooter = shooter;
        this.turret = turret;

        configureBindings();
    }

    public Command rotateTurretToJoystick(DoubleSupplier x, DoubleSupplier y) {
        return Commands.run(
                () -> {
                    double controlX = x.getAsDouble();
                    double controlY = y.getAsDouble();

                    if (Math.hypot(controlX, controlY) > 0.05) {
                        double angle = Math.atan2(controlY, controlX);
                        angle -= Math.PI / 2;
                        angle -= drivetrain.getState().Pose.getRotation().getRadians();
                        turret.setPositionWithRotation2d(new Rotation2d(wrapToPi(angle)));
                    }
                }, turret);
    }

    public Command rotateHoodToJoystick(DoubleSupplier y) {
        return Commands.run(
                () -> {
                    double controlY = y.getAsDouble();

                    hood.setPositionFromPercentage((controlY + 1) / 2);
                });
    }

    public Command snapTurretToHub() {
        return new InstantCommand(
            () -> turret.setPositionWithRotation2d(
                drivetrain.getAngleToScoreWhileMoving(
                    shooter.getTimeOfFlightFromDistanceMeters(
                        drivetrain.getDistanceToHub()
        ))), turret);
    }

    public Command snapHoodToHub() {
        return Commands.run(
            () -> {
                hood.setPosition(hood.getHoodHeightFromMetersToHub(drivetrain.getDistanceToHubWhileMoving(shooter.getTimeOfFlightFromDistanceMeters(drivetrain.getDistanceToHub()))));
            }, 
            hood);
    }

    public Command spinUpShooterForHubShot() {
        return Commands.run(
                () -> {
                    double distanceToHub = drivetrain.getDistanceToHubWhileMoving(
                            shooter.getTimeOfFlightFromDistanceMeters(
                                    drivetrain.getDistanceToHub()));
                    shooter.setVelocity(shooter.getShooterSpeedFromDistanceMeters(distanceToHub));
                },
                shooter);
    }

    public Command snapToPass() {
        return Commands.run(
            () -> {
                double angle;
                
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    angle = wrapToPi(-drivetrain.getState().Pose.getRotation().getRadians());
                } else {
                    angle = wrapToPi(-drivetrain.getState().Pose.getRotation().getRadians() - Math.PI);
                }

                turret.setPositionWithRotation2d(new Rotation2d(angle));
                hood.setPosition(-4.1);
                shooter.setVelocity(-50);
            }, 
            turret, hood, shooter);
    }

    public static double wrapToPi(double angle) {
        angle = (angle + Math.PI) % (2.0 * Math.PI);
        if (angle < 0) {
            angle += 2.0 * Math.PI;
        }
        return angle - Math.PI;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                {if (DriverStation.getAlliance().isPresent()) {
                            if (DriverStation.getAlliance().get() == Alliance.Red) {
                                return drive.withVelocityX(joystick.getLeftY() * MaxSpeed) 
                                    .withVelocityY(joystick.getLeftX() * MaxSpeed) 
                                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); 
                            } else {
                                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); 
                            }
                        } else {
                            return drive.withVelocityX(0) 
                                .withVelocityY(0) 
                                .withRotationalRate(0); 
                        }}
            )
        );

        turret.setDefaultCommand(snapTurretToHub());
        hood.setDefaultCommand(snapHoodToHub());

        joystick.leftBumper().whileTrue(spinUpShooterForHubShot()).onFalse(new InstantCommand(() -> shooter.setVelocity(0)));
        joystick.rightBumper().whileTrue(snapToPass()).onFalse(new InstantCommand(() -> shooter.setVelocity(0)));

        cojoystick.leftBumper().whileTrue(rotateTurretToJoystick(() -> cojoystick.getLeftX(), () -> -cojoystick.getLeftY()));
        cojoystick.leftBumper().whileTrue(rotateHoodToJoystick(() -> -cojoystick.getRightY()));

        joystick.a().onTrue(new InstantCommand(() -> shooter.requestVelocity(-40)));
        joystick.b().onTrue(new InstantCommand(() -> shooter.requestVelocity(-45)));
        joystick.x().onTrue(new InstantCommand(() -> shooter.requestVelocity(-55)));
        joystick.y().onTrue(new InstantCommand(() -> shooter.requestVelocity(-90)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> intake.setDutyCycle(-0.5))).onFalse(new InstantCommand(() -> intake.stopMotor()));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> intakePivot.setPosition(-16.5))).onFalse(new InstantCommand(() -> intakePivot.setPosition(-2)));
        
        joystick.rightTrigger().onTrue(new InstantCommand(() -> dyeRotor.setDutyCycle(1))).onFalse(new InstantCommand(() -> dyeRotor.stopMotor()));

        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
