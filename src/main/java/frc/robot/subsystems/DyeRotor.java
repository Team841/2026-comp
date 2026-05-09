// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class DyeRotor extends SubsystemBase {
    public TalonFX rotorMotor = new TalonFX(SuperstructureConstants.IDs.dyeRotorMotorID, "rio");

    public enum RotorState {
        STOP,
        FULLSPEED_FORWARD,
        LOWSPEED_FORWARD,
        UNJAM_BACKWARD
    }

    public RotorState rotorState = RotorState.STOP;

    public DyeRotor() {
        rotorMotor.getConfigurator().apply(SuperstructureConstants.DyeRotorConstants.dyeRotorMotorConfigs);
    }

    public void setState(RotorState wantedState) {
        rotorState = wantedState;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("DyeRotor/MotorStatorCurrent", this.rotorMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("DyeRotor/MotorVelocity", this.rotorMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("DyeRotor/Temp", this.rotorMotor.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("DyeRotor/State", rotorState);

        switch (rotorState) {
            case STOP:
                rotorMotor.stopMotor();
                break;

            case FULLSPEED_FORWARD:
                rotorMotor.set(1);
                break;

            case LOWSPEED_FORWARD:
                rotorMotor.set(0.2);
                break;

            case UNJAM_BACKWARD:
                rotorMotor.set(-0.3);
                break;
        
            default:
                rotorMotor.stopMotor();
                break;
        }
    }
}
