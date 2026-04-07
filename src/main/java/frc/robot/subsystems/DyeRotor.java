// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class DyeRotor extends SubsystemBase {
    public TalonFX rotorMotor = new TalonFX(SuperstructureConstants.IDs.dyeRotorMotorID, "rio");

    public DyeRotor() {
        rotorMotor.getConfigurator().apply(SuperstructureConstants.DyeRotorConstants.dyeRotorMotorConfigs);
    }

    public void setDutyCycle(double speed) {
        this.rotorMotor.set(speed);
    }

    public void stopMotor() {
        this.rotorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("DyeRotor/MotorStatorCurrent", this.rotorMotor.getStatorCurrent().getValueAsDouble());
    }
}
