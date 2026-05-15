// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Turret extends SubsystemBase {

    public TalonFX turretMotor = new TalonFX(SuperstructureConstants.IDs.turretMotorID, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private Rotation2d targetAngle = new Rotation2d();

    private Rotation2d overrideSupplierAngle = new Rotation2d();

    private Autoaim autoaim;

    StatusCode[] latestStatus;

    public enum TurretState {
        STOP,
        HOLD,
        TRACK_TARGET,
        TRACK_SUPPLIER
    }

    public TurretState turretState = TurretState.HOLD;

    public Turret(Autoaim autoaim) {
        turretMotor.getConfigurator().apply(SuperstructureConstants.TurretConstants.turretMotorConfigs);
        this.zero();
        this.autoaim = autoaim;
    }

    private void setPosition(Rotation2d angle) {
        this.targetAngle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), Math.cos(angle.getRadians())));
    }

    public void setOverridePosition(Rotation2d angle) {
        this.overrideSupplierAngle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), Math.cos(angle.getRadians())));
    }

    public void setState(TurretState wantedState) {
        this.turretState = wantedState;
    }

    public Rotation2d getTurretTargetAngle() {
        return this.targetAngle;
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(-this.turretMotor.getPosition().getValueAsDouble() * (Math.PI / 6.25));
    }

    public Rotation2d getTurretAngleAbsolute() {
        return new Rotation2d(Math.atan2(Math.sin(this.getTurretAngle().getRadians()), Math.cos(this.getTurretAngle().getRadians())));
    }

    public double getTurretVelocityInRadiansPerSecond() {
        return -this.turretMotor.getVelocity().getValueAsDouble() * (Math.PI / 6.25);
    }

    public double getContinuousTurretSetpoint() {

        double targetRadians = this.getTurretTargetAngle().getRadians();
        double ROTATIONS_PER_TURRET_REV = 12.5;
        double MIN_ROT = -5;
        double MAX_ROT = 5;

        double currentRot = turretMotor.getPosition().getValueAsDouble();

        double targetRot = targetRadians * (ROTATIONS_PER_TURRET_REV / (2 * Math.PI));

        double base = Math.round((currentRot - targetRot) / ROTATIONS_PER_TURRET_REV);
        double candidate = targetRot + base * ROTATIONS_PER_TURRET_REV;

        if (candidate > MAX_ROT) {
            candidate -= ROTATIONS_PER_TURRET_REV;
        } 
        else if (candidate < MIN_ROT) {
            candidate += ROTATIONS_PER_TURRET_REV;
        }

        return -candidate;
    }

    public boolean atAngle(double toleranceDegrees) {
        return Math.abs(this.getTurretTargetAngle().getDegrees() - this.getTurretAngleAbsolute().getDegrees()) < toleranceDegrees;
    }

    public boolean atAngleToFire() {
        return Math.abs(autoaim.getTurretRelativeAngleToFireWhileMoving().getDegrees() - this.getTurretAngleAbsolute().getDegrees()) < 10;
    }

    public void zero() {
        this.turretMotor.setPosition(0);
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
                this.turretMotor.setControl(control)
        };
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Turret/TargetPosition", this.getTurretTargetAngle().getDegrees());
        Logger.recordOutput("Turret/CurrentPosition", this.getTurretAngle().getDegrees());
        Logger.recordOutput("Turret/CurrentAbsolutePosition", this.getTurretAngleAbsolute().getDegrees());
        Logger.recordOutput("Turret/TargetMotorPosition", this.getContinuousTurretSetpoint());
        Logger.recordOutput("Turret/WithinTolerance10deg", this.atAngle(10));
        Logger.recordOutput("Turret/WithinToleranceToFire", this.atAngleToFire());
        Logger.recordOutput("Turret/State", turretState);

        switch (turretState) {
            case STOP:
                turretMotor.stopMotor();
                break;

            case HOLD:
                break;

            case TRACK_TARGET:
                this.setPosition(autoaim.getTurretRelativeAngleToFireWhileMoving().minus(autoaim.getTurretLeadAngleFromDrivetrainRotation()));
                break;

            case TRACK_SUPPLIER:
                this.setPosition(overrideSupplierAngle);
                break;
        
            default:
                turretMotor.stopMotor();
                break;
        }

        if (!turretState.equals(TurretState.STOP)) {
            this.latestStatus = this.setControl(positionControl.withPosition(this.getContinuousTurretSetpoint()));
        }
    }
}
