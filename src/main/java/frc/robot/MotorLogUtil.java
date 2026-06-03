package frc.robot;

import static edu.wpi.first.units.Units.Fahrenheit;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class MotorLogUtil {

    public static void logMotor(TalonFX motor, String subsystemName, String motorName) {
        Logger.recordOutput(subsystemName + "/" + motorName + "/Position", motor.getPosition().getValueAsDouble());
        Logger.recordOutput(subsystemName + "/" + motorName + "/Velocity", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput(subsystemName + "/" + motorName + "/Acceleration", motor.getAcceleration().getValueAsDouble());
        Logger.recordOutput(subsystemName + "/" + motorName + "/StatorCurrentAmps", motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(subsystemName + "/" + motorName + "/SupplyCurrentAmps", motor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(subsystemName + "/" + motorName + "/TempFahrenheit", motor.getDeviceTemp().getValue().in(Fahrenheit));
    }
}
