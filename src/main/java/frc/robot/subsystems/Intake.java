// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor = new TalonFX(SuperstructureConstants.IDs.intakeRollerMotorID, "rio");

    public Intake() {
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void setDutyCycle(double rps) {
        this.intakeMotor.set(rps);
    }

    public void stopMotor() {
        this.intakeMotor.stopMotor();
    }

    public double getIntakeVelocity() {
        return this.intakeMotor.getRotorVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {

    }
}
