package org.firstinspires.ftc.teamcode.wolfdrive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "Test")
@Disabled
@Config
public class MaxVelStrafeTest extends LinearOpMode {
    public static double SECONDS = 2;
    public static double POWER = 1.0;

    private double highestVelocityY = 0; // this is robot relative (sideways) and not field centric.

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            telemetry.addData("WARNING", "This test will (I think?) strafe right at %.2f power for %.2f seconds.", Range.clip(POWER, 0, 1), SECONDS);

            telemetry.addData("TODO", "Double check that you're strafing the correct direction and not running into a wall.");
            telemetry.addData("TIP", "It may help to collect this number by strafing both left and right.");
            telemetry.update();
        }

        timer.reset();

        // apply full power
        POWER = Range.clip(POWER, 0, 1);

//        drive.leftFront.setPower(POWER); drive.rightFront.setPower(-POWER);
//        drive.leftBack.setPower(-POWER);  drive.rightBack.setPower(POWER);

        // track velocity until timer is up
        while (opModeIsActive() && timer.seconds() < SECONDS) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, POWER), 0));
            PoseVelocity2d currentVelocity = drive.updatePoseEstimate();

            if (Math.abs(currentVelocity.linearVel.y) > highestVelocityY) {
                highestVelocityY = Math.abs(currentVelocity.linearVel.y);
            }

            telemetry.addData("Highest Velocity Y", highestVelocityY);
            telemetry.update();
            Log.d("Max_Velocity_Y_Test", "Highest Velocity Y: " + highestVelocityY);
        }

        // turn off motors
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, POWER), 0));
//        drive.leftFront.setPower(0); drive.rightFront.setPower(0);
//        drive.leftBack.setPower(0);  drive.rightBack.setPower(0);

        // leave display up
        while (opModeIsActive()) {
            telemetry.addData("Highest Velocity Y", highestVelocityY);
            telemetry.update();
            idle();
        }
    }
}