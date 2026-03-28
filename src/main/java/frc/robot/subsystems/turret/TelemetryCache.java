package frc.robot.subsystems.turret;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

public class TelemetryCache {

    public static AtomicReference<SwerveDrivetrain.SwerveDriveState> telemetryCache = new AtomicReference<>();
    public static Consumer<SwerveDrivetrain.SwerveDriveState> telemetryConsumer =
            swerveDriveState -> telemetryCache.set(swerveDriveState);
}
