package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.InfluxDbLogger;

@Config
@Autonomous(name = "RisingTidesAuto", group = "Rising Tides")
// @Disabled
public class RisingTidesAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Pose2d startingPose = new Pose2d(-63, 36, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);
        InfluxDbLogger influxDbLogger = new InfluxDbLogger(hardwareMap);
        // Claw claw = new Claw(hardwareMap);
        // Lift lift = new Lift(hardwareMap);

        // Vision here that outputs position
        int visionOutputPosition = 0;

        Action trajectoryAction0;
        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

        // @formatter:off
        trajectoryAction0 = drive.actionBuilder(startingPose)
                .lineToX(58)
                .lineToX(-60)
                .strafeTo(new Vector2d(-60,12))
                .setTangent(Math.toRadians(0))
                .lineToX(58)
                .lineToX(-60)
                .strafeToConstantHeading(new Vector2d(-60,58))
                .setTangent(Math.toRadians(0))
                .lineToX(58)
                .build();
//        trajectoryAction1 = drive.actionBuilder(drive.pose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5,30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
//                .build();
//        trajectoryAction2 = drive.actionBuilder(drive.pose)
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3)
//                .build();
//        trajectoryAction3 = drive.actionBuilder(drive.pose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3)
//                .build();
//        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d(48, 12))
//                .build();
        // @formatter:on

        // Actions that need to happen on init; for instance, a claw tightening
        // Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting position", startPosition);
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        Action trajectoryActionChosen;
        // if (startPosition == 0)
        // {
        trajectoryActionChosen = trajectoryAction0;
        // }
        // else if (startPosition == 1)
        // {
        // trajectoryActionChosen = trajectoryAction1;
        // }
        // else if (startPosition == 2)
        // {
        // trajectoryActionChosen = trajectoryAction2;
        // }
        // else
        // {
        // trajectoryActionChosen = trajectoryAction3;
        // }

        // @formatter:off
        Actions.runBlocking(
            new ParallelAction(
                new SequentialAction(
                    trajectoryActionChosen
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown(),
//                        trajectoryActionCloseOut,
                        ),
                    (telemetryPacket) -> { // Run some action
                        telemetry.addData("Run Time", runtime);
                        telemetry.addData("X", "%4.2f", drive.pose.position.x);
                        telemetry.addData("Y", "%4.2f", drive.pose.position.y);
                        telemetry.addData("Heading (deg)", "%4.2f", Math.toDegrees(drive.pose.heading.toDouble()));
                        telemetry.addData("Power Front Left/Right", "%4.2f, %4.2f", drive.leftFront.getPower(), drive.rightFront.getPower());
                        telemetry.addData("Power Back  Left/Right", "%4.2f, %4.2f", drive.leftBack.getPower(), drive.rightBack.getPower());
                        telemetry.addData("InfluxDB", influxDbLogger.isConnected()? "Connected" : "Disconnected");
                        telemetry.update();
                        return true; // Returning true causes the action to run again, returning false causes it to cease
                    }
                )
        );
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction( // several actions being run in parallel
//                                trajectoryActionChosen,
//                                (telemetryPacket) -> { // Run some action
//                                    telemetry.addData("Position", drive.pose);
//                                    telemetry.update();
//                                    return false; // Returning true causes the action to run again, returning false causes it to cease
//                                }
//                        ),
//                        trajectoryActionChosen
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown(),
//                        trajectoryActionCloseOut
//                )
//        );
        // @formatter:on
    }

}
