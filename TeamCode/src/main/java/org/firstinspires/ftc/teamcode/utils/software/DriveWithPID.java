package org.firstinspires.ftc.teamcode.utils.software;


import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.control.PIDFController;

import kotlin.jvm.functions.Function2;
public class DriveWithPID {
    private MecanumDrive drive;
    public static PIDCoefficients pid = new PIDCoefficients(0.00085,0.0, 0.00005);
    public static PIDCoefficients par_pid = new PIDCoefficients(0.0011,0.0, 0.00002);
    public static PIDCoefficients turn_pid = new PIDCoefficients(6.0,0.0, 0.00001);
    private PIDFController perp_pidfController = new PIDFController(pid, 0.0, 0.0);
    private PIDFController par_pidfController = new PIDFController(par_pid, 0.0, 0.0);
    private PIDFController turn_pidfController = new PIDFController(turn_pid, 0.0, 0.0);
    private int targetPosition = 0;
    private int internalOffset = 0;
    private int tolerance = 25;
    private long maxElapsedTime = 500;
    private double maxPower = 0.6;
    private DriveDirection direction;
    private Long startTime = null;
    public enum DriveDirection {
        STRAIGHT,
        STRAFE,
        TURN
    }

    public DriveWithPID(MecanumDrive drive, PIDCoefficients pid, DriveDirection direction) {
        this(drive, pid, direction, (x, v) -> 0.0);
    }

    public DriveWithPID(MecanumDrive drive, PIDCoefficients pid, DriveDirection direction, Function2<Double, Double, Double> f) {
        this.drive = drive;
        if(pid != null) {
            this.pid = pid;
        }

        this.perp_pidfController = new PIDFController(this.pid, 0, 0, 0, f);
        this.direction = direction;

        if(direction == DriveDirection.STRAFE) {
            internalOffset =  ((TwoDeadWheelLocalizer)drive.localizer).perp.getPositionAndVelocity().position;
        }
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    /**
     * Updates the power sent to the motor according to the pidf controller.
     */
    public void update() {
        drive.updatePoseEstimate();
        int perp_encoderTicks = ((TwoDeadWheelLocalizer) drive.localizer).perp.getPositionAndVelocity().position;;
        int par_encoderTicks = ((TwoDeadWheelLocalizer) drive.localizer).par.getPositionAndVelocity().position;
        double angle = ((TwoDeadWheelLocalizer) drive.localizer).imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        Log.d("DriveWithPID_Logger_1_update", "perp_encoderTicks:" + perp_encoderTicks
                + "| par_encoderTicks: " + par_encoderTicks
                + "| angle: " + String.format("%3.2f", Math.toDegrees(angle))
                + " | startTime: " + this.startTime);

        double perp_newPower = Range.clip(this.perp_pidfController.update(perp_encoderTicks), -maxPower, maxPower);
        double par_newPower = Range.clip(this.par_pidfController.update(par_encoderTicks), -maxPower, maxPower);
        double turn_newPower = Range.clip(this.turn_pidfController.update(angle), -maxPower, maxPower);
        double input_x = par_newPower;
        double input_y = perp_newPower;

        Vector2d input = new Vector2d(input_x, input_y);

        if (isBusy()) {
            drive.setDrivePowers(new PoseVelocity2d(input, turn_newPower));
            Log.d("DriveWithPID_Logger_2_update", "perp_newPower: " + String.format("%3.3f",input_y) + ", perp_lastError: " + perp_pidfController.getLastError());
            Log.d("DriveWithPID_Logger_2_update", "par_newPower: " + String.format("%3.3f",input_x) + ", par_lastError: " + par_pidfController.getLastError());
            Log.d("DriveWithPID_Logger_2_update", "turn_newPower: " + String.format("%3.3f",turn_newPower) + ", turn_lastError: " + String.format("%3.6f",turn_pidfController.getLastError()));
        } else {
            Log.d("DriveWithPID_Logger_3_done",
                    "perp_lastError: " + perp_pidfController.getLastError() +
                          ", par_lastError: " + par_pidfController.getLastError() +
                          ", turn_lastError: " + String.format("%3.6f",turn_pidfController.getLastError())
            );

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0,0.0), 0.0));
        }
    }

    /**
     * Update the PID values in the controller.
     * Note that it is not possible to replace f after instantiation
     * @param newPID the new pid values to use
     */
    public void setPIDCoefficients(PIDCoefficients newPID) {
        this.pid.kP = newPID.kP;
        this.pid.kI = newPID.kI;
        this.pid.kD = newPID.kD;
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold there at. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * target position, {@link #isBusy()} will return true.
     *
     * @param position the desired encoder target position
     */
    public void setTargetPosition(int position) {
        this.internalOffset = ((TwoDeadWheelLocalizer)drive.localizer).perp.getPositionAndVelocity().position;
        this.targetPosition = position + this.internalOffset;
        this.perp_pidfController.setTargetPosition(this.targetPosition);
        this.par_pidfController.setTargetPosition(((TwoDeadWheelLocalizer)drive.localizer).par.getPositionAndVelocity().position);
        this.turn_pidfController.setTargetPosition(((TwoDeadWheelLocalizer)drive.localizer).imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        this.setStartTime();
        Log.d("DriveWithPID_Logger_0_1_set", "position: " + position + ", this.targetPosition: "
                + this.targetPosition + ", this.internalOffset:" + this.internalOffset);
    }

    private class TargetPositionAction implements Action {
        int position;
        boolean blocking;

        public TargetPositionAction(int position, boolean blocking) {
            this.position = position;
            this.blocking = blocking;
            Log.d("DriveWithPID_Logger_0_init", "Current set Target Position: " + getSetTargetPosition() + " | " + "new set Position:" + position);
            setTargetPosition(position);

            Log.d("DriveWithPID_Logger_0_init", "new internal Target Position: " + getInternalTargetPosition());
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (blocking) {
                update();
                if(!isBusy()) {
                    resetStartTime();
                    Log.d("DriveWithPID_Logger_3_done", "Finished internal target: " + getInternalTargetPosition() + " | " + "current:" + getCurrentPosition());
                }
                return isBusy();
            }
            return false;
        }
    }

    /**
     * Creates an action that will call setTargetPosition with the provided position
     *
     * @param position the desired encoder target position
     */
    public Action setTargetPositionAction(int position) {
        return new TargetPositionAction(position, false);
    }

    /**
     * Creates an action that will call setTargetPosition with the provided position and
     * wait for the position to be reached
     *
     * @param position the desired encoder target position
     */
    public Action setTargetPositionActionBlocking(int position) {
        return new TargetPositionAction(position, true);
    }

    public void resetIntegralGain() {
        this.perp_pidfController.reset();
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    public boolean isBusy() {
        if(startTime == null) {
            return false;
        }
        return Math.abs(perp_pidfController.getLastError()) > tolerance && (System.currentTimeMillis() - startTime) < maxElapsedTime;
    }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     */
    public int getSetTargetPosition() {
        return this.targetPosition - this.internalOffset;
    }

    public int getInternalTargetPosition() {
        return this.targetPosition;
    }

    public int getCurrentPosition() {
        if(direction == DriveDirection.STRAFE) {
            return ((TwoDeadWheelLocalizer)drive.localizer).perp.getPositionAndVelocity().position;
        }
        return 0;
    }

    /**
     * Sets the maximum power level that can be sent to the motor
     * @param maxPower the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    /**
     * Returns the maximum power level that can be sent to the motor
     * @return the maximum level of the motor, a value in the interval [0.0, 1.0]
     */
    public double getMaxPower() {
        return maxPower;
    }

    public void setTargetPositionTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    public int getTargetPositionTolerance() {
        return tolerance;
    }

    public void setMaxElapsedTime(long maxElapsedTime) {
        this.maxElapsedTime = maxElapsedTime;
    }

    private void setStartTime() {
        if(this.startTime == null) {
            this.startTime = System.currentTimeMillis();
        }
    }

    private void resetStartTime() {
        this.startTime = null;
    }
}