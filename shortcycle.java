/*
This case is where we drop off an element, do the duck wheel, then go and park along the wall
(Except on the blue side instead of red side this time.)
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name="shortcycle", group="Tutorials")

public class shortcycle extends LinearOpMode {






    double lowerruntime = 0;
    double upperruntime = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightslide = null;
    private DcMotorEx leftslide = null;
    private DcMotorEx arm = null;
    private DcMotorEx rightarm = null;
    private Servo turn=null;
    private Servo claw1=null;
    private Servo claw2=null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST1 = 0; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 3; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 6; // Tag ID 18 from the 36h11 family
    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }


    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm  = hardwareMap.get(DcMotorEx.class, "leftarm");
        leftslide  = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide  = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        turn = hardwareMap.get(Servo.class, "turn");
        turn.setPosition(0.2);
        claw1.setPosition(0.1);//claw close
        claw2.setPosition(0.8);
        sleep(500);
        rightslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setTargetPosition(500);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslide.setPower(1);

        leftslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);

        leftslide.setTargetPosition(500);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(-5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(camera, 10);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    telemetry.addData("tag  ",tag.id);
                    telemetry.update();
                    if(tag.id == ID_TAG_OF_INTEREST1)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        telemetry.addData("CENTER ","PARKING POSITION");
                        telemetry.update();
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST2)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        telemetry.addData("RIGHT ","PARKING POSITION");
                        telemetry.update();
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        telemetry.addData("LEFT ","PARKING POSITION");
                        telemetry.update();
                        break;
                    }
                    else {

                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if(tagOfInterest.pose.x <= 20)
            {
                telemetry.addLine("Autonomous A");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(-40, -62, Math.toRadians(-180));
                drive.setPoseEstimate(start);

                TrajectorySequence move_to_first_deposit = drive.trajectorySequenceBuilder(start)
                        .strafeRight(62)
                        .splineToLinearHeading(new Pose2d(-29,3, Math.toRadians(-180)),Math.toRadians(150))
                        .turn(Math.toRadians(50))
                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(3.6, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })

                        .build();

                TrajectorySequence pick_1 = drive.trajectorySequenceBuilder(move_to_first_deposit.end())
                        .splineTo(new Vector2d(-44,-7),Math.toRadians(-180))
                        .splineTo(new Vector2d(-61,-7),Math.toRadians(-180))

//                        .forward(17.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(550); // 550
                            leftslide.setTargetPosition(550); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.7, () -> {
                            pickup();
                        })

                        .build();
                Trajectory drop_1 = drive.trajectoryBuilder(pick_1.end(),true)
                        .lineToConstantHeading(new Vector2d(-56,-24))
//                        .forward(3)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .build();

                TrajectorySequence pick_2 = drive.trajectorySequenceBuilder(drop_1.end())
                        .lineToConstantHeading(new Vector2d(-61,-67))

//                        .forward(16.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(0.8);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(360); // 550
                            leftslide.setTargetPosition(360); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            drop();
                        })
                        .addTemporalMarker(1.8, () -> {
                            pickup();
                        })
                        .addTemporalMarker(2.9, () -> {
                            upslide();
                        })
                        .build();
                Trajectory drop_2 = drive.trajectoryBuilder(pick_2.end(),true)
                        .splineTo(new Vector2d(-26,-11),Math.toRadians(50))

                        .addTemporalMarker(1.5, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();


                TrajectorySequence pick_3 = drive.trajectorySequenceBuilder(drop_2.end())
                        .splineTo(new Vector2d(-45,-14),Math.toRadians(-180))
                        .splineTo(new Vector2d(-60.5,-14),Math.toRadians(-180))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(200); // 550
                            leftslide.setTargetPosition(200); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.4, () -> {
                            pickup();
                        })
                        .addTemporalMarker(3, () -> {
                            rightslide.setTargetPosition(1000); // 550
                            leftslide.setTargetPosition(1000); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);                        })
                        .build();
                Trajectory drop_3 = drive.trajectoryBuilder(pick_3.end(),true)
                        .lineToConstantHeading(new Vector2d(-57 ,-28))
                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();

                TrajectorySequence pick_4 = drive.trajectorySequenceBuilder(drop_3.end())
//                        .splineTo(new Vector2d(-45,-17),Math.toRadians(-180))
                        .lineToConstantHeading(new Vector2d(-60,-24))
//                        .forward(3)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .build();

                drive.followTrajectorySequence(move_to_first_deposit);
                sleep(400);
                rightslide.setTargetPosition(1800);
                leftslide.setTargetPosition(1800);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);


                drive.followTrajectorySequence(pick_1);
                upslide();
                drive.followTrajectory(drop_1);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_2);

                drive.followTrajectory(drop_2);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_3);

                drive.followTrajectory(drop_3);

                rightslide.setTargetPosition(0);
                leftslide.setTargetPosition(0);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drop();
                sleep(200);
                drive.followTrajectorySequence(pick_4);

//

                return;
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                telemetry.addLine("Autonomous B");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(-40, -62, Math.toRadians(-180));
                drive.setPoseEstimate(start);

                TrajectorySequence move_to_first_deposit = drive.trajectorySequenceBuilder(start)
                        .strafeRight(62)
                        .splineToLinearHeading(new Pose2d(-29,3, Math.toRadians(-180)),Math.toRadians(150))
                        .turn(Math.toRadians(50))
                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(3.6, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })

                        .build();

                TrajectorySequence pick_1 = drive.trajectorySequenceBuilder(move_to_first_deposit.end())
                        .splineTo(new Vector2d(-44,-7),Math.toRadians(-180))
                        .splineTo(new Vector2d(-61,-7),Math.toRadians(-180))

//                        .forward(17.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(550); // 550
                            leftslide.setTargetPosition(550); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.7, () -> {
                            pickup();
                        })

                        .build();
                Trajectory drop_1 = drive.trajectoryBuilder(pick_1.end(),true)
//                    .splineToLinearHeading(new Pose2d(-26,-8,Math.toRadians(-130)),Math.toRadians(90))
                        .splineTo(new Vector2d(-26.5,-8),Math.toRadians(50))
                        .addTemporalMarker(2, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();

                TrajectorySequence pick_2 = drive.trajectorySequenceBuilder(drop_1.end())
                        .splineTo(new Vector2d(-46,-12),Math.toRadians(-180))
                        .splineTo(new Vector2d(-62.5,-12),Math.toRadians(-180))
//                        .forward(16.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(0.8);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(360); // 550
                            leftslide.setTargetPosition(360); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            drop();
                        })
                        .addTemporalMarker(2.2, () -> {
                            pickup();
                        })
                        .addTemporalMarker(2.9, () -> {
                            upslide();
                        })
                        .build();
                Trajectory drop_2 = drive.trajectoryBuilder(pick_2.end(),true)
                        .splineTo(new Vector2d(-26,-11),Math.toRadians(50))

                        .addTemporalMarker(1.5, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();


                TrajectorySequence pick_3 = drive.trajectorySequenceBuilder(drop_2.end())
                        .splineTo(new Vector2d(-45,-14),Math.toRadians(-180))
                        .splineTo(new Vector2d(-60.5,-14),Math.toRadians(-180))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(200); // 550
                            leftslide.setTargetPosition(200); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.4, () -> {
                            pickup();
                        })
                        .addTemporalMarker(3, () -> {
                            upslide();
                        })
                        .build();
                Trajectory drop_3 = drive.trajectoryBuilder(pick_3.end(),true)
                        .splineTo(new Vector2d(-29,-10),Math.toRadians(40))
                        .addTemporalMarker(1.5, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();

                TrajectorySequence pick_4 = drive.trajectorySequenceBuilder(drop_3.end())
//                        .splineTo(new Vector2d(-45,-17),Math.toRadians(-180))
                        .lineToConstantHeading(new Vector2d(-29,-14))
//                        .forward(3)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0, () -> {
                            rightslide.setTargetPosition(0); // 550
                            leftslide.setTargetPosition(0); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .build();

                drive.followTrajectorySequence(move_to_first_deposit);
                sleep(400);
                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);


                drive.followTrajectorySequence(pick_1);
                upslide();
                drive.followTrajectory(drop_1);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_2);

                drive.followTrajectory(drop_2);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_3);

                drive.followTrajectory(drop_3);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drop();
                sleep(200);
                drive.followTrajectorySequence(pick_4);

//

                return;            }
            else if(tagOfInterest.pose.x >= 50)
            {
                telemetry.addLine("Autonomous A");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(-40, -62, Math.toRadians(-180));
                drive.setPoseEstimate(start);

                TrajectorySequence move_to_first_deposit = drive.trajectorySequenceBuilder(start)
                        .strafeRight(62)
                        .splineToLinearHeading(new Pose2d(-29,3, Math.toRadians(-180)),Math.toRadians(150))
                        .turn(Math.toRadians(50))
                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(3.6, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })

                        .build();

                TrajectorySequence pick_1 = drive.trajectorySequenceBuilder(move_to_first_deposit.end())
                        .splineTo(new Vector2d(-44,-7),Math.toRadians(-180))
                        .splineTo(new Vector2d(-61,-7),Math.toRadians(-180))

//                        .forward(17.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(550); // 550
                            leftslide.setTargetPosition(550); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.7, () -> {
                            pickup();
                        })

                        .build();
                Trajectory drop_1 = drive.trajectoryBuilder(pick_1.end(),true)
//                    .splineToLinearHeading(new Pose2d(-26,-8,Math.toRadians(-130)),Math.toRadians(90))
                        .splineTo(new Vector2d(-26.5,-8),Math.toRadians(50))
                        .addTemporalMarker(2, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();

                TrajectorySequence pick_2 = drive.trajectorySequenceBuilder(drop_1.end())
                        .splineTo(new Vector2d(-46,-12),Math.toRadians(-180))
                        .splineTo(new Vector2d(-62.5,-12),Math.toRadians(-180))
//                        .forward(16.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(0.8);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(360); // 550
                            leftslide.setTargetPosition(360); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            drop();
                        })
                        .addTemporalMarker(2.2, () -> {
                            pickup();
                        })
                        .addTemporalMarker(2.9, () -> {
                            upslide();
                        })
                        .build();
                Trajectory drop_2 = drive.trajectoryBuilder(pick_2.end(),true)
                        .splineTo(new Vector2d(-26,-11),Math.toRadians(50))

                        .addTemporalMarker(1.5, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .build();


                TrajectorySequence pick_3 = drive.trajectorySequenceBuilder(drop_2.end())
                        .splineTo(new Vector2d(-45,-14),Math.toRadians(-180))
                        .splineTo(new Vector2d(-60.5,-14),Math.toRadians(-180))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.3, () -> {
                            rightslide.setTargetPosition(200); // 550
                            leftslide.setTargetPosition(200); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .addTemporalMarker(2.4, () -> {
                            pickup();
                        })
                        .addTemporalMarker(3, () -> {
                            rightslide.setTargetPosition(1000); // 550
                            leftslide.setTargetPosition(1000); // 550
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);                        })
                        .build();
                Trajectory drop_3 = drive.trajectoryBuilder(pick_3.end(),true)
                        .back(40) //-14
                        .splineToLinearHeading(new Pose2d(-27,-35,Math.toRadians(-180)),Math.toRadians(-90))
                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
//                    .splineTo(new Vector2d(-16, -5), Math.toRadians(210))
                        .build();

                TrajectorySequence pick_4 = drive.trajectorySequenceBuilder(drop_3.end())
//                        .splineTo(new Vector2d(-45,-17),Math.toRadians(-180))
                        .lineToConstantHeading(new Vector2d(-60,-24))
//                        .forward(3)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.5, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })
                        .build();

                drive.followTrajectorySequence(move_to_first_deposit);
                sleep(400);
                rightslide.setTargetPosition(1800);
                leftslide.setTargetPosition(1800);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);


                drive.followTrajectorySequence(pick_1);
                upslide();
                drive.followTrajectory(drop_1);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_2);

                drive.followTrajectory(drop_2);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(200);

                drive.followTrajectorySequence(pick_3);

                drive.followTrajectory(drop_3);

                uparm();
                sleep(2000);
                rightslide.setTargetPosition(2000);
                leftslide.setTargetPosition(2000);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                drop();
                sleep(500);
                pickup();
                arm.setTargetPosition(0);
                arm.setPower(0.75);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(0); // 550
                leftslide.setTargetPosition(2); // 550
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
//

                return;
            }
        }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */





    }
//    public void autoA(){
//
////        drive.followTrajectorySequence(dk);
//    }
    public void upslide(){
        rightslide.setTargetPosition(2500);
        leftslide.setTargetPosition(2500);
        rightslide.setPower(1);
        leftslide.setPower(1);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void uparm(){
        arm.setTargetPosition(-300);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void down(){

        rightslide.setTargetPosition(100);
        leftslide.setTargetPosition(100);
        rightslide.setPower(1);
        leftslide.setPower(1);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(0.1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void pickup(){
        claw1.setPosition(0.05);//claw close
        claw2.setPosition(0.85);
    }
    public void drop(){
        claw1.setPosition(0.3);//claw open
        claw2.setPosition(0.6); // 0.5

    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}