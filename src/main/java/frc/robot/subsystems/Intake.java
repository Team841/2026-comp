// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Intake extends SubsystemBase {
    public TalonFX intakeMotorLeft = new TalonFX(SuperstructureConstants.IDs.intakeRollerLeftMotorID, "rio");
    public TalonFX intakeMotorRight = new TalonFX(SuperstructureConstants.IDs.intakeRollerRightMotorID, "rio");

    private Follower follower = new Follower(SuperstructureConstants.IDs.intakeRollerLeftMotorID, MotorAlignmentValue.Opposed);

    public enum IntakeState {
        STOP,
        INTAKE,
        FULLSPEED_INTAKE,
        OUTTAKE
    }

    public IntakeState intakeState = IntakeState.STOP;

    public Intake() {
        this.intakeMotorLeft.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive)).withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));
        this.intakeMotorRight.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive)).withAudio(new AudioConfigs().withAllowMusicDurDisable(true)));
        this.intakeMotorRight.setControl(follower);
    }

    public void setState(IntakeState wantedState) {
        intakeState = wantedState;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Velocity", this.intakeMotorLeft.getVelocity().getValueAsDouble());
        Logger.recordOutput("Intake/RightMotorTemp", this.intakeMotorRight.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Intake/LeftMotorTemp", this.intakeMotorLeft.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Intake/State", intakeState);

        switch (intakeState) {
            case STOP:
                intakeMotorLeft.stopMotor();
                break;

            case INTAKE:
                intakeMotorLeft.set(0.6);
                break;

            case FULLSPEED_INTAKE:
                intakeMotorLeft.set(1);
                break;

            case OUTTAKE:
                intakeMotorLeft.set(-1);
                break;
        
            default:
                intakeMotorLeft.stopMotor();
                break;
        }
    }
}
