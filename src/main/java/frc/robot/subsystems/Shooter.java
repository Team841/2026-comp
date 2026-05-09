// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Shooter extends SubsystemBase {
	public TalonFX rightMotor = new TalonFX(SuperstructureConstants.IDs.rightShooterMotorID, "rio");
	public TalonFX leftMotor = new TalonFX(SuperstructureConstants.IDs.leftShooterMotorID, "rio");

	private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
	private Follower follower = new Follower(SuperstructureConstants.IDs.rightShooterMotorID, MotorAlignmentValue.Opposed);

	private InterpolatingDoubleTreeMap shooterSpeedsMap;
	private InterpolatingDoubleTreeMap passingShooterSpeedsMap;

	private Autoaim autoaim;

	private double targetVelocity = 0;

	private double overrideSupplierVelocity = 0;

	StatusCode[] latestStatus;

	public enum ShooterState {
        STOP,
        HOLD,
        FOLLOW_TARGET,
        FOLLOW_SUPPLIER
    }

	public ShooterState shooterState = ShooterState.STOP;

	public Shooter(Autoaim autoaim) {
		this.rightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.shooterMotorConfigs);
		this.leftMotor.setControl(follower);

		this.autoaim = autoaim;

		this.shooterSpeedsMap = new InterpolatingDoubleTreeMap();
		this.shooterSpeedsMap.put(1.0, ShooterSpeed.M1.getRPS());
		this.shooterSpeedsMap.put(2.0, ShooterSpeed.M2.getRPS());
		this.shooterSpeedsMap.put(3.0, ShooterSpeed.M3.getRPS());
		this.shooterSpeedsMap.put(4.0, ShooterSpeed.M4.getRPS());
		this.shooterSpeedsMap.put(5.0, ShooterSpeed.M5.getRPS());
		this.shooterSpeedsMap.put(6.0, ShooterSpeed.M6.getRPS());
		this.shooterSpeedsMap.put(7.0, ShooterSpeed.M7.getRPS());

		this.passingShooterSpeedsMap = new InterpolatingDoubleTreeMap();
		this.passingShooterSpeedsMap.put(0.0, PassingShooterSpeed.M0.getRPS());
		this.passingShooterSpeedsMap.put(4.0, PassingShooterSpeed.M4.getRPS());
		this.passingShooterSpeedsMap.put(8.0, PassingShooterSpeed.M8.getRPS());
		this.passingShooterSpeedsMap.put(10.0, PassingShooterSpeed.M10.getRPS());
		this.passingShooterSpeedsMap.put(12.0, PassingShooterSpeed.M12.getRPS());
		this.passingShooterSpeedsMap.put(16.0, PassingShooterSpeed.M16.getRPS());
	}

	public void setState(ShooterState wantedState) {
		this.shooterState = wantedState;
	}

	public double getShooterSpeedFromDistanceMeters(double distance) {
		return this.shooterSpeedsMap.get(distance);
	}

	public double getShooterPassingSpeedFromDistanceMeters(double distance) {
		return this.passingShooterSpeedsMap.get(distance);
	}

	public void requestOverrideVelocity(double newRPS) {
		if (newRPS == this.overrideSupplierVelocity) {
			this.overrideSupplierVelocity = 0;
		} else {
			this.overrideSupplierVelocity = newRPS;
		}
	}

	public void setOverrideVelocity(double rps) {
		this.overrideSupplierVelocity = rps;
	}

	public void stopMotor() {
		this.targetVelocity = 0;
		this.rightMotor.stopMotor();
	}

	public StatusCode[] setControl(ControlRequest control) {
		return new StatusCode[] {
				this.rightMotor.setControl(control)
		};
	}

	public boolean atfullSpeed() {
		if (Math.abs(this.targetVelocity - this.getShooterVelocity()) < 3 && this.targetVelocity != 0) {
			return true;
		}
		if (Math.abs(this.targetVelocity) > 70 && Math.abs(this.targetVelocity - this.getShooterVelocity()) < 10) {
			return true;
		}
		return false;
	}

	public double getShooterVelocity() {
		return this.rightMotor.getRotorVelocity().getValueAsDouble();
	}

	public double getShooterTargetVelocity() {
		return this.targetVelocity;
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Shooter/TargetSpeed", this.getShooterTargetVelocity());
		Logger.recordOutput("Shooter/Speed", this.getShooterVelocity());
		Logger.recordOutput("Shooter/AtFullSpeed", this.atfullSpeed());
		Logger.recordOutput("Shooter/State", shooterState);

        switch (shooterState) {
            case STOP:
                this.stopMotor();
                break;

            case HOLD:
                break;

            case FOLLOW_TARGET:
				if (autoaim.target.equals(Autoaim.FiringLocation.HUB)) {
					targetVelocity = getShooterSpeedFromDistanceMeters(autoaim.getDistanceToScoreWhileMoving());
				} else if (autoaim.target.equals(Autoaim.FiringLocation.PASS)) {
					targetVelocity = getShooterPassingSpeedFromDistanceMeters(autoaim.getDistanceToScoreWhileMoving());
				}
                break;

            case FOLLOW_SUPPLIER:
				targetVelocity = overrideSupplierVelocity;
                break;
        
            default:
                this.stopMotor();
                break;
        }

        if (!shooterState.equals(ShooterState.STOP)) {
			if (this.targetVelocity == 0) {
				this.stopMotor();
			} else {
				this.latestStatus = this.setControl(velocityControl.withVelocity(this.targetVelocity));
			}
        }
	}

	public enum ShooterSpeed {

		M1(-36),
		M2(-41.5),
		M3(-46.5),
		M4(-51),
		M5(-54),
		M6(-58),
		M7(-61);

		private final double rps;

		ShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}

	public enum PassingShooterSpeed {

		M0(-10),
		M2(-28),
		M4(-39),
		M6(-51),
		M8(-60),
		M10(-68),
		M12(-81),
		M14(-94),
		M16(-100);

		private final double rps;

		PassingShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}
}
