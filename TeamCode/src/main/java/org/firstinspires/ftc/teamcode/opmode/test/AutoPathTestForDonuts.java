package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.software.AutoActionScheduler;

@Config
@Autonomous(group = "Test")
public final class AutoPathTestForDonuts extends LinearOpMode {

    public static Pose2d starting = new Pose2d(16, 62, -Math.PI/2);
 //   public static Pose2d starting = new Pose2d(48.5, -35.5, Math.PI);

    //   public static Pose2d starting = new Pose2d(16.0, -63.0, Math.PI/2);

    public static Pose2d backdrop = new Pose2d(48.5, 35.5, -Math.PI);

    public static Pose2d spike = new Pose2d(27.0, 24.0, -Math.PI);


    protected AutoActionScheduler sched;

    protected Intake intake;
    protected Outtake outtake;

    protected MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, starting);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        this.sched = new AutoActionScheduler(this::update);

        intake.initialize(true);
        outtake.initialize();

        waitForStart();

        // 1. drive to the backstage
        // 2. raise the slide and prepare the yellow pixel drop
        // 3. score the yellow pixel onto the backdrop
        // 4. retract slide
        // 5. drive to the spike position
        // 6. drop the purple pixel
        // 7. drive to parking
        sched.addAction(
            new SequentialAction(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .splineTo(spike.position, -Math.PI/2)
                            .build(),
                    new SleepAction(0.5),
                    drive.actionBuilder(backdrop)
                            //.setTangent(0)
                            .strafeTo(spike.position)
                            .build(),
                    new SleepAction(3)
            )
        );

        sched.run();

        while(!isStopRequested()) {
            telemetry.addData("Game timer: ", getRuntime());
            idle();
        }

    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
        outtake.update();
        intake.update();
    }
}