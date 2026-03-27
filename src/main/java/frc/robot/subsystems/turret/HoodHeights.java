package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public enum HoodHeights {
    M1(0),
    M5(-2),
    M10(-4.1);

    private final double position;
    public static final InterpolatingDoubleTreeMap hoodHeightMap = new InterpolatingDoubleTreeMap();

    static {
        hoodHeightMap.put(1.0, HoodHeights.M1.getPosition());
        hoodHeightMap.put(5.0, HoodHeights.M5.getPosition());
        hoodHeightMap.put(10.0, HoodHeights.M10.getPosition());
    }

    HoodHeights(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
