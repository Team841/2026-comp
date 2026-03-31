// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotorLeft = new TalonFX(SuperstructureConstants.IDs.intakeRollerLeftMotorID, "rio");
    private TalonFX intakeMotorRight = new TalonFX(SuperstructureConstants.IDs.intakeRollerRightMotorID, "rio");

    private Follower follower = new Follower(SuperstructureConstants.IDs.intakeRollerLeftMotorID, MotorAlignmentValue.Opposed);

    public Intake() {
        this.intakeMotorLeft.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive)));
        this.intakeMotorRight.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive)));
        this.intakeMotorRight.setControl(follower);
    }

    public void setDutyCycle(double speed) {
        this.intakeMotorLeft.set(speed);
    }

    public void stopMotor() {
        this.intakeMotorLeft.stopMotor();
    }


}
