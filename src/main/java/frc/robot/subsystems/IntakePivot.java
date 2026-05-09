// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class IntakePivot extends SubsystemBase {

    public TalonFX intakePivotMotor = new TalonFX(SuperstructureConstants.IDs.intakePivotMotorID, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private double targetPosition = -3;

    StatusCode[] latestStatus;

    public enum IntakePivotState {
        STOP,
        MATCHSTART_STOW,
        COMPACT_STOW,
        BUMP_STOW,
        INTAKE
    }

    public IntakePivotState intakePivotState = IntakePivotState.MATCHSTART_STOW;

    public IntakePivot() {
        intakePivotMotor.getConfigurator().apply(SuperstructureConstants.IntakePivotConstants.intakePivotMotorConfigs);
    }

    public void setState(IntakePivotState wantedState) {
        intakePivotState = wantedState;
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
            this.intakePivotMotor.setControl(control)
        };
    }

    public double getIntakePivotTargetPosition() {
        return this.targetPosition;
    }

    public double getIntakePivotPosition() {
        return this.intakePivotMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(double position) {
        return Math.abs(this.intakePivotMotor.getPosition().getValueAsDouble() - position) < 2;
    }

    public void zero() {
        this.intakePivotMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("IntakePivot/TargetPosition", this.getIntakePivotTargetPosition());
        Logger.recordOutput("IntakePivot/Position", this.getIntakePivotPosition());
        Logger.recordOutput("IntakePivot/State", intakePivotState);
        
        switch (intakePivotState) {
            case STOP:
                intakePivotMotor.stopMotor();
                break;

            case INTAKE:
                this.targetPosition = -22.9;
                break;

            case BUMP_STOW:
                this.targetPosition = -20;
                break;

            case MATCHSTART_STOW:
                this.targetPosition = -3;
                break;

            case COMPACT_STOW:
                this.targetPosition = -6;
                break;
        
            default:
                intakePivotMotor.stopMotor();
                break;
        }

        if (!intakePivotState.equals(IntakePivotState.STOP)) {
            this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));
        }
    }
}
