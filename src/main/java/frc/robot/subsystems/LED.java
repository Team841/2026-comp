package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj.DriverStation;

public class LED extends SubsystemBase{
    private RobotContainer robotContainer;

    private final AddressableLED LED = new AddressableLED(9);
    private final AddressableLEDBuffer Buffer = new AddressableLEDBuffer(19);

    // private final AddressableLEDBufferView BufferRight = Buffer.createView(4, 12);
    // private final AddressableLEDBufferView BufferLeft2 = Buffer.createView(0, 3);
    // private final AddressableLEDBufferView BufferLeft = Buffer.createView(13, 18).reversed();
   
    Color yellow = new Color(255, 0, 150);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 0, 255);
    Color blue = new Color(0, 255, 0);
    Color purple = new Color(255, 255, 0);
    Distance ledSpacing = Meters.of(1 / 120.0);

    LEDPattern baseYellowScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, yellow);
    LEDPattern patternYellowScroll = baseYellowScroll.scrollAtRelativeSpeed(Percent.per(Second).of(120));
    LEDPattern patternYellowScrollReverse = baseYellowScroll.scrollAtRelativeSpeed(Percent.per(Second).of(300)).reversed();

    LEDPattern baseYellowRedScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, red, yellow);
    LEDPattern patternYellowRedScroll = baseYellowRedScroll.scrollAtRelativeSpeed(Percent.per(Second).of(60));

    LEDPattern baseBlueScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, blue);
    LEDPattern patternBlueScroll = baseBlueScroll.scrollAtRelativeSpeed(Percent.per(Second).of(130));

    LEDPattern basePurpleScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, purple);
    LEDPattern patternPurpleScroll = basePurpleScroll.scrollAtRelativeSpeed(Percent.per(Second).of(120));

    LEDPattern baseRedScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, red);
    LEDPattern patternRedScroll = baseRedScroll.scrollAtRelativeSpeed(Percent.per(Second).of(300));
    
    LEDPattern baseRainbowScroll = LEDPattern.rainbow(255, 255);
    LEDPattern patternRainbowScroll = baseRainbowScroll.scrollAtRelativeSpeed(Percent.per(Second).of(300));

    LEDPattern baseRedBreathe = LEDPattern.solid(red);
    LEDPattern patternRedBreathe = baseRedBreathe.breathe(Second.of(0.15));

    LEDPattern baseRedBreatheScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, red);
    LEDPattern base2RedBreatheScroll = baseRedBreatheScroll.scrollAtRelativeSpeed(Percent.per(Second).of(100)).reversed();
    LEDPattern patternRedBreatheScroll = base2RedBreatheScroll.breathe(Second.of(2));
    
    LEDPattern baseBlueBreatheScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, blue);
    LEDPattern base2BlueBreatheScroll = baseBlueBreatheScroll.scrollAtRelativeSpeed(Percent.per(Second).of(100)).reversed();
    LEDPattern patternBlueBreatheScroll = base2BlueBreatheScroll.breathe(Second.of(2));

    LEDPattern patternYellowSolid = LEDPattern.solid(yellow);
    LEDPattern patternRedSolid = LEDPattern.solid(red);
    LEDPattern patternGreenSolid = LEDPattern.solid(green);
    LEDPattern patternBlueSolid = LEDPattern.solid(blue);
    LEDPattern patternBrownSolid = LEDPattern.solid(Color.kBrown);

    public LED(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        LED.setLength(Buffer.getLength());
        patternRedSolid.applyTo(Buffer);
        LED.setData(Buffer);
        LED.start();
    }

    private void apply(LEDPattern ledPattern) {
        // ledPattern.applyTo(BufferLeft);
        // ledPattern.applyTo(BufferRight);
        // ledPattern.applyTo(BufferLeft2);
        ledPattern.applyTo(Buffer);
        LED.setData(Buffer);
    }

    @Override
    public void periodic() {
        if (!DriverStation.isDSAttached()) {
            apply(patternYellowRedScroll);
        } else if (DriverStation.isDisabled()) {
            if (RobotConstants.isRedAlliance.getAsBoolean()) {
                apply(patternRedBreatheScroll);
            } else {
                apply(patternBlueBreatheScroll);
            }
        } else if (robotContainer.currentMode == RobotContainer.RobotMode.AUTOAIM_FIRE || robotContainer.currentMode == RobotContainer.RobotMode.PASS_SHOT) {
            if (!robotContainer.shooter.atfullSpeed()) {
                apply(patternRedScroll);
            } else if (!robotContainer.turret.atAngle(10)) {
                apply(patternRedBreathe);
            } else if (robotContainer.currentMode == RobotContainer.RobotMode.AUTOAIM_FIRE) {
                apply(patternGreenSolid);
            } else {
                apply(patternBlueSolid);
            }
        } else if (Math.abs(robotContainer.intake.intakeMotorLeft.getDutyCycle().getValueAsDouble()) > 0.1) {
            apply(patternYellowScrollReverse);
        } else if (DriverStation.isAutonomousEnabled()) {
            apply(patternPurpleScroll);
        } else {
            apply(patternYellowScroll);
        }
    }
}