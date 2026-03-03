// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Turret extends SubsystemBase {

    private TalonFX turretMotor = new TalonFX(SuperstructureConstants.IDs.turretMotorID, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private double targetPosition = 0;

    StatusCode[] latestStatus;

    public Turret() {
        turretMotor.getConfigurator().apply(SuperstructureConstants.TurretConstants.turretMotorConfigs);
    }

    public void setPosition(double position) {
        this.targetPosition = position;
    }

    public void setPositionWithRotation2d(Rotation2d angle) {
        this.targetPosition = -angle.getRotations() * 12.5;
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
                this.turretMotor.setControl(control)
        };
    }

    public double getTurretTargetPosition() {
        return this.targetPosition;
    }

    public double getPosition() {
        return this.turretMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition() {
        return Math.abs(this.getTurretTargetPosition() - this.getPosition()) < 0.2;
    }

    public void zero() {
        this.turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));
    }
}
