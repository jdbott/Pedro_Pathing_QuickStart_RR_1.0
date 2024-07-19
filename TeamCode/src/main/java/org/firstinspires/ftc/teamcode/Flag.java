package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlagServoThings;

public class Flag {
    private final Servo flagServo;

    public Flag(HardwareMap hardwareMap) {
        flagServo = hardwareMap.get(Servo.class, "flagServo");
        flagServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setManualPosition(double position) {
        flagServo.setPosition(position);
    }

    public void flagUp() {
        flagServo.setPosition(FlagServoThings.FLAG_UP);
    }

    public void flagDown() {
        flagServo.setPosition(FlagServoThings.FLAG_DOWN);
    }

    public double getPosition() {
        return flagServo.getPosition();
    }
}