package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivers.Lights.Color;
import org.firstinspires.ftc.teamcode.drivers.Lights.Direction;
import org.firstinspires.ftc.teamcode.drivers.Lights.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.Lights.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.drivers.Lights.PrismAnimations;

public class LEDController {

    private GoBildaPrismDriver prism;

    private enum LEDState {
        INIT_AUTO,
        INIT_TELE,
        NORMAL,
        AIMING,
        LOCKED,
        SHOOTING,
        ENDGAME
    }

    private LEDState currentState = null;

    public void initHardware(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    private void setState(LEDState newState) {
        if (newState == currentState) {
            return;
        }

        currentState = newState;
        prism.clearAllAnimations();

        switch (newState) {
            case INIT_TELE:
            case NORMAL:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(Color.YELLOW)
                );
                break;

            case AIMING:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        new PrismAnimations.Snakes(
                                6, 2, 15,
                                Color.TRANSPARENT,
                                1f,
                                Direction.Forward,
                                Color.WHITE
                        )
                );
                break;

            case LOCKED:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(Color.GREEN)
                );
                break;

            case SHOOTING:
            case ENDGAME:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        new PrismAnimations.Snakes(
                                4, 6, 20,
                                Color.TRANSPARENT,
                                1.0f,
                                Direction.Forward,
                                Color.YELLOW
                        )
                );
                break;

            case INIT_AUTO:
                break;
        }
    }

    public void initTele() {
        setState(LEDState.INIT_TELE);
    }

    public void normal() {
        setState(LEDState.NORMAL);
    }

    public void aiming() {
        setState(LEDState.AIMING);
    }

    public void locked() {
        setState(LEDState.LOCKED);
    }

    public void shooting() {
        setState(LEDState.SHOOTING);
    }

    public void endgame() {
        setState(LEDState.ENDGAME);
    }

    public void initAuto(boolean isBlueAlliance) {
        if (currentState == LEDState.INIT_AUTO) {
            return;
        }

        currentState = LEDState.INIT_AUTO;
        prism.clearAllAnimations();

        Color allianceColor = isBlueAlliance ? Color.BLUE : Color.RED;
        prism.insertAndUpdateAnimation(
                LayerHeight.LAYER_0,
                new PrismAnimations.Solid(allianceColor)
        );
    }

    public void setBallCount(int count) {
        // Indicator support is currently disabled; keep the method for callers.
    }
}
