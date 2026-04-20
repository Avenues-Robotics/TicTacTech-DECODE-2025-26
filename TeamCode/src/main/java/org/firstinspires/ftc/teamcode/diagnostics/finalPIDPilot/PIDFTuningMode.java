package org.firstinspires.ftc.teamcode.diagnostics.finalPIDPilot;

public enum PIDFTuningMode {
    REV_UP("Reach target as fast as possible. Accepts transient overshoot."),
    MAINTAIN("Hold target against disturbances. Prioritizes zero steady-state error.");

    private final String description;

    PIDFTuningMode(String description) {
        this.description = description;
    }

    public String description() {
        return description;
    }
}
