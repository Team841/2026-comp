// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.SuperstructureConstants;

public class Turret extends SubsystemBase {

    private TalonFX turretMotor = new TalonFX(SuperstructureConstants.IDs.turretMotorID, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private Rotation2d targetAngle = new Rotation2d();

    StatusCode[] latestStatus;

    public Turret() {
        turretMotor.getConfigurator().apply(SuperstructureConstants.TurretConstants.turretMotorConfigs);
    }

    public void setPosition(Rotation2d angle) {
        this.targetAngle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), Math.cos(angle.getRadians())));
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

    public double getContinuousTurretSetpoint() {

        double targetRadians = this.getTurretTargetAngle().getRadians();
        double ROTATIONS_PER_TURRET_REV = 12.5;
        double MIN_ROT = -5;
        double MAX_ROT = 5;

        double currentRot = turretMotor.getPosition().getValueAsDouble();

        // Convert radians → turret rotations
        double targetRot = targetRadians * (ROTATIONS_PER_TURRET_REV / (2 * Math.PI));

        // Find closest equivalent revolution
        double base = Math.round((currentRot - targetRot) / ROTATIONS_PER_TURRET_REV);
        double candidate = targetRot + base * ROTATIONS_PER_TURRET_REV;

        // If outside limits, jump one revolution
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

    public static double shortestAngleDifference(double current, double target) {
        double diff = target - current;
        return Math.atan2(Math.sin(diff), Math.cos(diff));
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
        this.latestStatus = this.setControl(positionControl.withPosition(this.getContinuousTurretSetpoint()));
        Logger.recordOutput("Turret/TargetPosition", this.getTurretTargetAngle().getDegrees());
        Logger.recordOutput("Turret/CurrentPosition", this.getTurretAngle().getDegrees());
        Logger.recordOutput("Turret/CurrentAbsolutePosition", this.getTurretAngleAbsolute().getDegrees());
        Logger.recordOutput("Turret/TargetMotorPosition", this.getContinuousTurretSetpoint());
        Logger.recordOutput("Turret/WithinTolerance", this.atAngle(10));


    }
}
