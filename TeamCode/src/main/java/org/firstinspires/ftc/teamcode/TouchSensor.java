package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TouchSensor {
    private final DigitalChannel touchSensor;

    public TouchSensor(HardwareMap hardwareMap) {
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState() {
        return !touchSensor.getState();
    }
}