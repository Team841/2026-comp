// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Hood extends SubsystemBase {

    public TalonFX hoodMotor = new TalonFX(SuperstructureConstants.IDs.hoodMotorID, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private double targetPosition = 0;

    private double overrideSupplierPosition = 0;

    InterpolatingDoubleTreeMap hoodHeightMap;

    private Autoaim autoaim;

    StatusCode[] latestStatus;

    public enum HoodState {
        STOP,
        HOLD,
        TRACK_TARGET,
        TRACK_SUPPLIER
    }

    public HoodState hoodState = HoodState.HOLD;

    public Hood(Autoaim autoaim) {
        hoodMotor.getConfigurator().apply(SuperstructureConstants.HoodConstants.hoodMotorConfigs);

        this.autoaim = autoaim;

        this.hoodHeightMap = new InterpolatingDoubleTreeMap();
        this.hoodHeightMap.put(1.0, HoodHeight.M1.getPosition());
        this.hoodHeightMap.put(5.0, HoodHeight.M5.getPosition());
        this.hoodHeightMap.put(10.0, HoodHeight.M10.getPosition());

        this.zero();
    }

    public void setState(HoodState wantedState) {
        hoodState = wantedState;
    }

    public void setOverrideSupplier(double position) {
        overrideSupplierPosition = position;
    }

    public double getHoodHeightFromMetersToHub(double distance) {
        return this.hoodHeightMap.get(distance);
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
                this.hoodMotor.setControl(control)
        };
    }

    public double getHoodTargetPosition() {
        return this.targetPosition;
    }

    public double getHoodPosition() {
        return this.hoodMotor.getPosition().getValueAsDouble();
    }

    public void zero() {
        this.hoodMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));
        Logger.recordOutput("Hood/TargetPosition", this.getHoodTargetPosition());
        Logger.recordOutput("Hood/Position", this.getHoodPosition());
        Logger.recordOutput("Hood/State", hoodState);

        switch (hoodState) {
            case STOP:
                hoodMotor.stopMotor();
                break;

            case HOLD:
                break;

            case TRACK_TARGET:
                targetPosition = this.getHoodHeightFromMetersToHub(autoaim.getDistanceToScoreWhileMoving());
                break;

            case TRACK_SUPPLIER:
                targetPosition = overrideSupplierPosition;
                break;
        
            default:
                hoodMotor.stopMotor();
                break;
        }

        if (!hoodState.equals(HoodState.STOP)) {
            this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));
        }
    }

    public enum HoodHeight {

		M1(0),
		M5(-2),
        M10(-4.1);

		private final double position;

		HoodHeight(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}
}
