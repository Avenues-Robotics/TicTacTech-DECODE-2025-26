package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallDetector {

    private ColorSensor sensor1, sensor2, sensor3;

    // 🔧 TUNE THESE PER SENSOR
    private static final int ON_THRESHOLD_1 = 350;
    private static final int OFF_THRESHOLD_1 = 250;

    private static final int ON_THRESHOLD_2 = 350;
    private static final int OFF_THRESHOLD_2 = 250;

    private static final int ON_THRESHOLD_3 = 350;
    private static final int OFF_THRESHOLD_3 = 250;

    // Sensor states (hysteresis memory)
    private boolean s1Active = false;
    private boolean s2Active = false;
    private boolean s3Active = false;

    // Debounce system
    private int currentCount = 0;
    private int stableCount = 0;
    private int stableCycles = 0;

    private static final int REQUIRED_STABLE_CYCLES = 3;

    // Debug values
    private int i1, i2, i3;

    public void init(HardwareMap hardwareMap) {
        sensor1 = hardwareMap.get(ColorSensor.class, "color1");
        sensor2 = hardwareMap.get(ColorSensor.class, "color2");
        sensor3 = hardwareMap.get(ColorSensor.class, "color3");
    }

    public void update() {

        // Read intensities
        i1 = getIntensity(sensor1);
        i2 = getIntensity(sensor2);
        i3 = getIntensity(sensor3);

        // Apply hysteresis
        s1Active = applyHysteresis(i1, s1Active, ON_THRESHOLD_1, OFF_THRESHOLD_1);
        s2Active = applyHysteresis(i2, s2Active, ON_THRESHOLD_2, OFF_THRESHOLD_2);
        s3Active = applyHysteresis(i3, s3Active, ON_THRESHOLD_3, OFF_THRESHOLD_3);

        int count = 0;
        if (s1Active) count++;
        if (s2Active) count++;
        if (s3Active) count++;

        // Debounce (stability filter)
        if (count == currentCount) {
            stableCycles++;
        } else {
            currentCount = count;
            stableCycles = 0;
        }

        if (stableCycles >= REQUIRED_STABLE_CYCLES) {
            stableCount = currentCount;
        }
    }

    private int getIntensity(ColorSensor sensor) {
        return sensor.red() + sensor.green() + sensor.blue();
    }

    private boolean applyHysteresis(int value, boolean currentState, int onThreshold, int offThreshold) {

        if (!currentState && value > onThreshold) {
            return true;  // turn ON
        }

        if (currentState && value < offThreshold) {
            return false; // turn OFF
        }

        return currentState; // hold state
    }

    public int getBallCount() {
        return stableCount;
    }

    // 🔍 DEBUG OUTPUT
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Ball Count", stableCount);

        telemetry.addData("S1 Intensity", i1);
        telemetry.addData("S2 Intensity", i2);
        telemetry.addData("S3 Intensity", i3);

        telemetry.addData("S1 Active", s1Active);
        telemetry.addData("S2 Active", s2Active);
        telemetry.addData("S3 Active", s3Active);
    }
}