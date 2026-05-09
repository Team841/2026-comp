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

    private double targetPosition = 2;

    InterpolatingDoubleTreeMap hoodHeightMap;

    StatusCode[] latestStatus;

    public Hood() {
        hoodMotor.getConfigurator().apply(SuperstructureConstants.HoodConstants.hoodMotorConfigs);

        this.hoodHeightMap = new InterpolatingDoubleTreeMap();
        this.hoodHeightMap.put(1.0, HoodHeight.M1.getPosition());
        this.hoodHeightMap.put(5.0, HoodHeight.M5.getPosition());
        this.hoodHeightMap.put(10.0, HoodHeight.M10.getPosition());

        this.zero();
    }

    public void setPosition(double position) {
        this.targetPosition = position;
    }

    public void setPositionFromPercentage(double value) {
        this.targetPosition = -4.1 * value;
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
