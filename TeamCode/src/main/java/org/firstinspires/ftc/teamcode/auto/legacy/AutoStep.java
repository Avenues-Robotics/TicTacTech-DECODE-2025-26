package org.firstinspires.ftc.teamcode.auto.legacy;

public class AutoStep {
    public MoveType type = MoveType.DRIVE;
    public double value = 0;
    public double speed = 0;

    public AutoStep() {}

    public AutoStep(MoveType type, double value, double speed) {
        this.type = type;
        this.value = value;
        this.speed = speed;
    }
}
