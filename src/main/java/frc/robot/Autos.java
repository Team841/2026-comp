package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Autoaim.FiringLocation;
import frc.robot.subsystems.DyeRotor.RotorState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class Autos {

    private final RobotContainer robotContainer;
    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Autoaim autoaim;
    private final Shooter shooter;
    private final Turret turret;
    private final Hood hood;
    private final DyeRotor dyeRotor;
    private final Intake intake;
    private final IntakePivot intakePivot;
    
    public Autos(RobotContainer robotContainer, AutoFactory autoFactory) {
        this.robotContainer = robotContainer;
        this.autoFactory = autoFactory;

        this.drivetrain = robotContainer.drivetrain;
        this.autoaim = robotContainer.autoaim;
        this.shooter = robotContainer.shooter;
        this.turret = robotContainer.turret;
        this.hood = robotContainer.hood;
        this.dyeRotor = robotContainer.dyeRotor;
        this.intake = robotContainer.intake;
        this.intakePivot = robotContainer.intakePivot;
    }

    public AutoRoutine RightSideOneSweepNZ() {
        AutoRoutine routine = autoFactory.newRoutine("RightSideOneSweep");

        AutoTrajectory path = routine.trajectory("RightSideOneSweep");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("Shoot").onTrue(new ParallelCommandGroup(
                    Commands.runOnce(
                    () -> {
                        autoaim.setFiringLocation(FiringLocation.HUB);
                        turret.setState(TurretState.TRACK_TARGET);
                        hood.setState(HoodState.TRACK_TARGET);
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                    }, turret, shooter, hood),
                    robotContainer.runDyeRotorForHubShot()
                ));

        return routine;
    }

    public AutoRoutine LeftSideOneSweepNZ() {
        AutoRoutine routine = autoFactory.newRoutine("RightSideOneSweep");

        AutoTrajectory path = routine.trajectory("RightSideOneSweep").mirrorY();  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("Shoot").onTrue(new ParallelCommandGroup(
                    Commands.runOnce(
                    () -> {
                        autoaim.setFiringLocation(FiringLocation.HUB);
                        turret.setState(TurretState.TRACK_TARGET);
                        hood.setState(HoodState.TRACK_TARGET);
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                    }, turret, shooter, hood),
                    robotContainer.runDyeRotorForHubShot()
                ));

        return routine;
    }

    public AutoRoutine LeftSideOneSweepPlusDepotAndReturn() {
        AutoRoutine routine = autoFactory.newRoutine("LeftSideOneSweepPlusDepotAndReturn");

        AutoTrajectory path = routine.trajectory("LeftSideOneSweepPlusDepot");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                Commands.parallel(
                    path.cmd(),
                    Commands.runOnce(() -> {
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                        autoaim.setFiringLocation(FiringLocation.HUB);
                    })
                )
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeBumpStow").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
            }, turret, shooter, hood),
            robotContainer.runDyeRotorForHubShot()
        ));

        path.atTime("StopFire").onTrue(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.STOP);
            }, turret, shooter, hood, dyeRotor)
        );

        path.done().onTrue(
            Commands.runOnce(
                    () -> {
                        autoaim.setFiringLocation(FiringLocation.HUB);
                        turret.setState(TurretState.HOLD);
                        hood.setState(HoodState.HOLD);
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                        dyeRotor.setState(RotorState.STOP);
                    }, turret, shooter, hood, dyeRotor)
        );

        return routine;
    }

    public AutoRoutine LeftSideTwoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("LeftSideTwoSweep");

        AutoTrajectory path = routine.trajectory("LeftSideTwoSweep");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                Commands.parallel(
                    path.cmd(),
                    Commands.runOnce(() -> {
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                        autoaim.setFiringLocation(FiringLocation.HUB);
                        turret.setState(TurretState.TRACK_TARGET);
                        hood.setState(HoodState.TRACK_TARGET);
                    })
                )
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeBumpStow").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.FULLSPEED_FORWARD);
            }, turret, shooter, hood) //,
            // robotContainer.runDyeRotorForHubShot()
        ));

        path.atTime("StopFire").onTrue(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.STOP);
            }, turret, shooter, hood, dyeRotor)
        );

        path.atTime("DisableVision").onTrue(
            Commands.runOnce(
                () -> robotContainer.vision.disableVision(), 
                robotContainer.vision)
        );

        path.atTime("EnableVision").onTrue(
            Commands.runOnce(
                () -> robotContainer.vision.disableVision(), 
                robotContainer.vision)
        );

        return routine;
    }

    public AutoRoutine RightSideTwoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("LeftSideTwoSweep");

        AutoTrajectory path = routine.trajectory("LeftSideTwoSweep").mirrorY();  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                Commands.parallel(
                    path.cmd(),
                    Commands.runOnce(() -> {
                        shooter.setState(ShooterState.FOLLOW_TARGET);
                        autoaim.setFiringLocation(FiringLocation.HUB);
                        turret.setState(TurretState.TRACK_TARGET);
                        hood.setState(HoodState.TRACK_TARGET);
                    })
                )
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeBumpStow").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.FULLSPEED_FORWARD);
            }, turret, shooter, hood) //,
            // robotContainer.runDyeRotorForHubShot()
        ));

        path.atTime("StopFire").onTrue(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.STOP);
            }, turret, shooter, hood, dyeRotor)
        );

        path.atTime("DisableVision").onTrue(
            Commands.runOnce(
                () -> robotContainer.vision.disableVision(), 
                robotContainer.vision)
        );

        path.atTime("EnableVision").onTrue(
            Commands.runOnce(
                () -> robotContainer.vision.disableVision(), 
                robotContainer.vision)
        );

        return routine;
    }

    public AutoRoutine RightSideOneSweepOutpostAndReturn() {
        AutoRoutine routine = autoFactory.newRoutine("RightSideOneSweepOutpostAndReturn");

        AutoTrajectory path = routine.trajectory("RightSideOneSweepPlusOutpost");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeBumpStow").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.BUMP_STOW);
        }, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
            }, turret, shooter, hood),
            robotContainer.runDyeRotorForHubShot()
        ));

        path.atTime("StopFire").onTrue(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.STOP);
            }, turret, shooter, hood, dyeRotor)
        );

        return routine;
    }

    public AutoRoutine MiddlePreloadFire() {
        AutoRoutine routine = autoFactory.newRoutine("MiddlePreloadFire");

        AutoTrajectory path = routine.trajectory("MiddleScorePreload");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
            }, turret, shooter, hood),
            robotContainer.runDyeRotorForHubShot()
        ));

        path.atTime("StopFire").onTrue(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
                dyeRotor.setState(RotorState.STOP);
            }, turret, shooter, hood, dyeRotor)
        );

        return routine;
    }

    public AutoRoutine MiddleDepot() {
        AutoRoutine routine = autoFactory.newRoutine("MiddleDepot");

        AutoTrajectory path = routine.trajectory("MiddleDepot");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
            }, turret, shooter, hood),
            robotContainer.runDyeRotorForHubShot()
        ));

        return routine;
    }

    public AutoRoutine MiddleOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("MiddleOutpost");

        AutoTrajectory path = routine.trajectory("MiddleOutpost");  
        
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.atTime("IntakeDown").onTrue(Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKE);
            intakePivot.setState(IntakePivotState.INTAKE);
        }, intake, intakePivot));

        path.atTime("IntakeUp").onTrue(Commands.runOnce(() -> {
            intakePivot.setState(IntakePivotState.COMPACT_STOW);
        }, intakePivot));

        path.atTime("HubFire").onTrue(new ParallelCommandGroup(
            Commands.runOnce(
            () -> {
                autoaim.setFiringLocation(FiringLocation.HUB);
                turret.setState(TurretState.TRACK_TARGET);
                hood.setState(HoodState.TRACK_TARGET);
                shooter.setState(ShooterState.FOLLOW_TARGET);
            }, turret, shooter, hood),
            robotContainer.runDyeRotorForHubShot()
        ));

        return routine;
    }

}
