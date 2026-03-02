package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeBallDetector {
    private DcMotorEx intake;
    private int ballCount = 0;
    private static final double LOAD_THRESHOLD = 0.75;
    private boolean isStrained = false;
    private ElapsedTime debounceTimer = new ElapsedTime();

    public void init(DcMotorEx intake) {
        this.intake = intake;
    }

    public void update(double estimatedV) {
        if (Math.abs(estimatedV) < 0.5) {
            isStrained = false;
            return;
        }

        double currentVelocity = Math.abs(intake.getVelocity());
        double ratio = currentVelocity / Math.abs(estimatedV);

        if (ratio < (expectedRatio() * LOAD_THRESHOLD)) {
            if (!isStrained && debounceTimer.milliseconds() > 100) {
                ballCount++;
                isStrained = true;
            }
        } else {
            isStrained = false;
            debounceTimer.reset();
        }
    }

    private double expectedRatio() {
        return 150.0;
    }

    public int getBallCount() { return ballCount; }
    public void resetCount() { ballCount = 0; }
}