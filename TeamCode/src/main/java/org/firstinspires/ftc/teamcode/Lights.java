package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lights {
    private final RevBlinkinLedDriver lights;

    public Lights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
    }
}