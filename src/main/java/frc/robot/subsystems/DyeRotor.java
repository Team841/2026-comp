// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class DyeRotor extends SubsystemBase {
    private TalonFX rotorMotor = new TalonFX(SuperstructureConstants.IDs.dyeRotorMotorID, "rio");

    public DyeRotor() {
        rotorMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void setDutyCycle(double rps) {
        this.rotorMotor.set(rps);
    }

    public void stopMotor() {
        this.rotorMotor.stopMotor();
    }

    public double getIntakeVelocity() {
        return this.rotorMotor.getRotorVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {

    }
}
