//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot;
//
//import static edu.wpi.first.units.Units.*;
//
//import java.util.function.DoubleSupplier;
//
//import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.pathplanner.lib.auto.NamedCommands;
//import com.ctre.phoenix6.swerve.SwerveRequest;
//
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.constants.TunerConstants;
//import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.DyeRotor;
//import frc.robot.subsystems.Hood;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.IntakePivot;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.vision.Vision;
//import frc.robot.subsystems.vision.VisionIO;
//
//public class RobotContainer {
//    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
//    private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
//
//    /* Setting up bindings for necessary control of the swerve drive platform */
//    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
//            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
//
//    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
//    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//    private final SwerveRequest.ApplyRobotSpeeds speed = new SwerveRequest.ApplyRobotSpeeds();
//
//
//    private final Telemetry logger = new Telemetry(MaxSpeed);
//
//    private final CommandXboxController joystick = new CommandXboxController(0);
//    private final CommandXboxController cojoystick = new CommandXboxController(1);
//
//    public final Drivetrain drivetrain;
//    public final DyeRotor dyeRotor;
//    public final Hood hood;
//    public final Intake intake;
//    public final IntakePivot intakePivot;
//    public final Shooter shooter;
//    public final Turret turret;
//
//    public final VisionIO visionIO;
//    public final Vision vision;
//
//    public RobotMode currentMode = RobotMode.NEUTRAL;
//    private Command activeCommand = null;
//
//    public enum RobotMode {
//        NEUTRAL,
//        AUTOAIM_FIRE,
//        PASS_SHOT,
//        SPINUP_SHOOTER
//    }
//
//    public RobotContainer(Drivetrain drivetrain, DyeRotor dyeRotor, Hood hood, Intake intake, IntakePivot intakePivot, Shooter shooter, Turret turret, VisionIO visionIO, Vision vision) {
//        this.drivetrain = drivetrain;
//        this.dyeRotor = dyeRotor;
//        this.hood = hood;
//        this.intake = intake;
//        this.intakePivot = intakePivot;
//        this.shooter = shooter;
//        this.turret = turret;
//        this.visionIO = visionIO;
//        this.vision = vision;
//
//        NamedCommands.registerCommand("IntakeDownAndSpin", intakeDownAndSpin());
//        NamedCommands.registerCommand("IntakeUpAndStop", intakeUpAndStop());
//        NamedCommands.registerCommand("IntakeUpFullAndStop", intakeUpFullAndStop());
//        NamedCommands.registerCommand("IntakeForOutpost", new InstantCommand(() -> intakePivot.setPosition(-9.5)));
//
//        NamedCommands.registerCommand("AutoAimAndFire10Sec", autoAimAndFire().withTimeout(10)
//                                                            .finallyDo(() -> {shooter.setVelocity(0);
//                                                                            dyeRotor.stopMotor();}));
//
//        NamedCommands.registerCommand("AutoAimAndFire3Sec", autoAimAndFire().withTimeout(3)
//                                                            .finallyDo(() -> {shooter.setVelocity(0);
//                                                                            dyeRotor.stopMotor();}));
//
//        NamedCommands.registerCommand("AutoPassAndFire", autoPassAndFire().withTimeout(5));
//
//        NamedCommands.registerCommand("SpinUpShooterEarly", spinUpShooterEarly());
//
//        NamedCommands.registerCommand("ForceResetPose", new InstantCommand(() -> drivetrain.forceCameraPose(), drivetrain));
//
//        configureBindings();
//    }
//
//    public Command rotateTurretToJoystick(DoubleSupplier x, DoubleSupplier y) {
//        return Commands.run(
//                () -> {
//                    double controlX = x.getAsDouble();
//                    double controlY = y.getAsDouble();
//
//                    if (Math.hypot(controlX, controlY) > 0.05) {
//                        double angle = Math.atan2(controlY, controlX);
//                        angle -= Math.PI / 2;
//                        angle -= drivetrain.getState().Pose.getRotation().getRadians();
//                        turret.setPosition(new Rotation2d(wrapToPi(angle)));
//                    }
//                }, turret);
//    }
//
//    public Command rotateHoodToJoystick(DoubleSupplier y) {
//        return Commands.run(
//                () -> {
//                    double controlY = y.getAsDouble();
//
//                    hood.setPositionFromPercentage((controlY + 1) / 2);
//                });
//    }
//
//    public Command intakeDownAndSpin() {
//        return new InstantCommand(() -> {
//            intakePivot.setPosition(-17);
//            intake.setDutyCycle(-0.5);
//        }, intake, intakePivot);
//    }
//
//    public Command intakeUpAndStop() {
//        return new InstantCommand(() -> {
//            intakePivot.setPosition(-10);
//            intake.stopMotor();
//        }, intake, intakePivot);
//    }
//
//    public Command intakeUpFullAndStop() {
//        return new InstantCommand(() -> {
//            intakePivot.setPosition(-3);
//            intake.stopMotor();
//        }, intake, intakePivot);
//    }
//
//    public Command spinUpShooterEarly() {
//        return new InstantCommand(() -> {
//            shooter.setVelocity(-30);
//        }, shooter);
//    }
//
//    public Command snapTurretToHub() {
//        return new RunCommand(
//            () -> turret.setPosition(
//                drivetrain.getAngleToScoreWhileMoving(
//                    getIteratedTof()
//        )), turret);
//    }
//
//    public Command snapHoodToHub() {
//        return Commands.run(
//            () -> {
//                hood.setPosition(
//                    hood.getHoodHeightFromMetersToHub(
//                        drivetrain.getDistanceToHubWhileMoving(
//                            getIteratedTof()
//                        )));
//            },
//            hood);
//    }
//
//    public Command spinUpShooterForHubShot() {
//        return Commands.run(
//                () -> {
//                    double distanceToHubIterated =
//                        drivetrain.getDistanceToHubWhileMoving(getIteratedTof());
//                    shooter.setVelocity(shooter.getShooterSpeedFromDistanceMeters(distanceToHubIterated));
//                },
//                shooter);
//    }
//
//    public double getIteratedTof() {
//        return shooter.getTimeOfFlightFromDistanceMeters(
//            drivetrain.getDistanceToHubWhileMoving(
//                shooter.getTimeOfFlightFromDistanceMeters(
//                    drivetrain.getDistanceToHubWhileMoving(
//                        shooter.getTimeOfFlightFromDistanceMeters(
//                            drivetrain.getDistanceToHubWhileMoving(
//                                shooter.getTimeOfFlightFromDistanceMeters(
//                                    drivetrain.getDistanceToHubWhileMoving(
//                                        shooter.getTimeOfFlightFromDistanceMeters(
//                                            drivetrain.getDistanceToHubWhileMoving(
//                                                shooter.getTimeOfFlightFromDistanceMeters(
//                                                    drivetrain.getDistanceToHubWhileMoving(
//                                                        shooter.getTimeOfFlightFromDistanceMeters(
//                                                            drivetrain.getDistanceToHubWhileMoving(
//                                                                shooter.getTimeOfFlightFromDistanceMeters(
//                                                                    drivetrain.getDistanceToHubWhileMoving(
//                                                                        shooter.getTimeOfFlightFromDistanceMeters(
//                                                                            drivetrain.getDistanceToHub()
//                        )))))))))))))))));
//    }
//
//    public Command snapToPass() {
//        return Commands.run(
//            () -> {
//                turret.setPosition(drivetrain.getAngleToPassWhileMoving(getIteratedTof()));
//                hood.setPosition(-4.1);
//                    shooter.setVelocity(shooter.getShooterPassingSpeedFromDistanceMeters(drivetrain.getDistanceToDriverStationWall()));
//            },
//            turret, hood, shooter);
//    }
//
//    public Command runDyeRotorForHubShot() {
//        return Commands.run(
//            () -> {
//                if (turret.atAngle(10) && shooter.atfullSpeed()) {
//                    dyeRotor.setDutyCycle(0.8);
//                } else {
//                    dyeRotor.setDutyCycle(0);
//                }
//            },
//            dyeRotor);
//    }
//
//    public Command runDyeRotorForPassShot() {
//        return Commands.run(
//            () -> {
//                if (turret.atAngle(10) && shooter.atfullSpeed() && drivetrain.goodToPass()) {
//                    dyeRotor.setDutyCycle(0.8);
//                } else {
//                    dyeRotor.setDutyCycle(0);
//                }
//            },
//            dyeRotor);
//    }
//
//    public Command autoAimAndFire() {
//        return new ParallelCommandGroup(
//            spinUpShooterForHubShot(),
//            snapHoodToHub(),
//            snapTurretToHub(),
//            runDyeRotorForHubShot()
//        );
//    }
//
//    public Command autoPassAndFire() {
//        return new ParallelCommandGroup(
//            snapToPass(),
//            runDyeRotorForPassShot()
//        );
//    }
//
//    public static double wrapToPi(double angle) {
//        angle = (angle + Math.PI) % (2.0 * Math.PI);
//        if (angle < 0) {
//            angle += 2.0 * Math.PI;
//        }
//        return angle - Math.PI;
//    }
//
//    public void setMode(RobotMode newMode) {
//
//        if (newMode == currentMode) return;
//
//        if (activeCommand != null) {
//            activeCommand.cancel();
//        }
//
//        currentMode = newMode;
//
//        switch (currentMode) {
//
//            case NEUTRAL:
//                activeCommand = new InstantCommand(() -> {shooter.setVelocity(0); dyeRotor.stopMotor();}, shooter);
//                break;
//
//            case AUTOAIM_FIRE:
//                activeCommand = autoAimAndFire();
//                break;
//
//            case PASS_SHOT:
//                activeCommand = autoPassAndFire();
//                break;
//
//            case SPINUP_SHOOTER:
//                activeCommand = spinUpShooterForHubShot();
//        }
//
//        if (activeCommand != null) {
//            activeCommand.schedule();
//        }
//    }
//
//    public void toggleMode(RobotMode newMode) {
//
//        if (currentMode == newMode) {
//            setMode(RobotMode.NEUTRAL);
//        } else {
//            setMode(newMode);
//        }
//    }
//
//    private void configureBindings() {
//        drivetrain.setDefaultCommand(
//            drivetrain.applyRequest(() ->
//                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
//                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
//                .withRotationalRate(-joystick.getRightX() * MaxAngularRate))
//        );
//
//        // joystick.back().whileTrue(
//        //     new InstantCommand(() -> drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d()))).withTimeout(0.2)
//        //     .andThen(drivetrain.applyRequest(() -> speed.withSpeeds(new ChassisSpeeds(0.2, 0, 0))))
//        // ).onFalse(drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0)));
//
//        // turret.setDefaultCommand(snapTurretToHub().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
//        // hood.setDefaultCommand(snapHoodToHub().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
//
//        joystick.rightTrigger().onTrue(new InstantCommand(() -> toggleMode(RobotMode.AUTOAIM_FIRE)));
//        joystick.rightBumper().onTrue(new InstantCommand(() -> toggleMode(RobotMode.PASS_SHOT)));
//        joystick.x().onTrue(new InstantCommand(() -> setMode(RobotMode.NEUTRAL)));
//        joystick.povRight().onTrue(new InstantCommand(() -> toggleMode(RobotMode.SPINUP_SHOOTER)));
//
//        joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));
//
//        joystick.leftTrigger().onTrue(new RepeatCommand(new InstantCommand(() -> intake.setDutyCycle(-0.5), intake).onlyIf(() -> intakePivot.atPosition(-16.5)))).onFalse(new InstantCommand(() -> intake.stopMotor(), intake));
//        joystick.leftTrigger().onTrue(new InstantCommand(() -> intakePivot.setPosition(-16.5), intakePivot)).onFalse(new InstantCommand(() -> intakePivot.setPosition(-14), intakePivot));
//
//        joystick.rightStick().onTrue(new InstantCommand(() -> intakePivot.setPosition(-3), intakePivot));
//        joystick.leftBumper().onTrue(new InstantCommand(() -> dyeRotor.setDutyCycle(-0.3))).onFalse(new InstantCommand(() -> dyeRotor.stopMotor()));
//
//        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
//        joystick.back().onTrue(new InstantCommand(() -> drivetrain.forceCameraPose(), drivetrain));
//
//        joystick.povLeft().whileTrue(new RepeatCommand(new InstantCommand(() -> drivetrain.forceCameraPose())));
//
//        cojoystick.leftBumper().whileTrue(rotateTurretToJoystick(() -> cojoystick.getLeftX(), () -> -cojoystick.getLeftY()));
//        cojoystick.leftBumper().whileTrue(rotateHoodToJoystick(() -> -cojoystick.getRightY()));
//
//        cojoystick.a().onTrue(new InstantCommand(() -> shooter.requestVelocity(-40)));
//        cojoystick.b().onTrue(new InstantCommand(() -> shooter.requestVelocity(-45)));
//        cojoystick.x().onTrue(new InstantCommand(() -> shooter.requestVelocity(-55)));
//        cojoystick.y().onTrue(new InstantCommand(() -> shooter.requestVelocity(-90)));
//
//        drivetrain.registerTelemetry(logger::telemeterize);
//    }
//
//    public Command getAutonomousCommand() {
//        // Simple drive forward auton
//        final var idle = new SwerveRequest.Idle();
//        return Commands.sequence(
//            // Reset our field centric heading to match the robot
//            // facing away from our alliance station wall (0 deg).
//            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
//            // Then slowly drive forward (away from us) for 5 seconds.
//            drivetrain.applyRequest(() ->
//                drive.withVelocityX(0.5)
//                    .withVelocityY(0)
//                    .withRotationalRate(0)
//            )
//            .withTimeout(5.0),
//            // Finally idle for the rest of auton
//            drivetrain.applyRequest(() -> idle)
//        );
//    }
//}
