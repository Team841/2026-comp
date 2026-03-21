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
	private TalonFX rightMotor = new TalonFX(SuperstructureConstants.IDs.rightShooterMotorID, "rio");
	private TalonFX leftMotor = new TalonFX(SuperstructureConstants.IDs.leftShooterMotorID, "rio");

	private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
	private Follower follower = new Follower(SuperstructureConstants.IDs.rightShooterMotorID, MotorAlignmentValue.Opposed);

	private InterpolatingDoubleTreeMap shooterSpeedsMap;

	private InterpolatingDoubleTreeMap passingShooterSpeedsMap;

	private InterpolatingDoubleTreeMap timeOfFlightMap;

	private double targetVelocity = 0;

	StatusCode[] latestStatus;

	public Shooter() {
		this.rightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.shooterMotorConfigs);
		this.leftMotor.setControl(follower);

		this.targetVelocity = 0;

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

		this.timeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.timeOfFlightMap.put(1.0, TimeOfFlight.M1.getSeconds());
		this.timeOfFlightMap.put(2.0, TimeOfFlight.M2.getSeconds());
		this.timeOfFlightMap.put(3.0, TimeOfFlight.M3.getSeconds());
		this.timeOfFlightMap.put(4.0, TimeOfFlight.M4.getSeconds());
		this.timeOfFlightMap.put(5.0, TimeOfFlight.M5.getSeconds());

	}

	public void setVelocity(double rps) {
		this.targetVelocity = rps;
	}

	public double getShooterSpeedFromDistanceMeters(double distance) {
		return this.shooterSpeedsMap.get(distance);
	}

	public double getShooterPassingSpeedFromDistanceMeters(double distance) {
		return this.passingShooterSpeedsMap.get(distance);
	}

	public double getTimeOfFlightFromDistanceMeters(double distance) {
		return this.timeOfFlightMap.get(distance);
	}

	public void requestVelocity(double newRPS) {
		if (newRPS == this.targetVelocity) {
			this.targetVelocity = 0;
		} else {
			this.targetVelocity = newRPS;
		}
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
		return Math.abs(this.targetVelocity - this.getShooterVelocity()) < 2 && this.targetVelocity != 0;
	}

	public double getShooterVelocity() {
		return this.rightMotor.getRotorVelocity().getValueAsDouble();
	}

	public double getShooterTargetVelocity() {
		return this.targetVelocity;
	}

	@Override
	public void periodic() {
		if (this.targetVelocity == 0) {
			this.stopMotor();
		} else {
			this.latestStatus = this.setControl(velocityControl.withVelocity(this.targetVelocity));
		}

		Logger.recordOutput("Shooter/TargetSpeed", this.getShooterTargetVelocity());
		Logger.recordOutput("Shooter/Speed", this.getShooterVelocity());
		Logger.recordOutput("Shooter/AtFullSpeed", this.atfullSpeed());
	}

	public enum ShooterSpeed {

		M1(-24 * 3/2),
		M2(-27 * 3/2),
		M3(-31 * 3/2),
		M4(-35 * 3/2),
		M5(-38 * 3/2),
		M6(-41 * 3/2),
		M7(-43 * 3/2);

		private final double rps;

		ShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}

	public enum PassingShooterSpeed {

		M0(-1),
		M2(-14),
		M4(-25),
		M6(-37),
		M8(-46),
		M10(-54),
		M12(-67),
		M14(-80),
		M16(-95);

		private final double rps;

		PassingShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}

	public enum TimeOfFlight {

		M1(0.8),
		M2(1),
		M3(1.2),
		M4(1.4),
		M5(1.6),
		M6(1.8),
		M7(2);

		private final double sec;

		TimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}

}
