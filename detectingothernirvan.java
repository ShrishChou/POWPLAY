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
@Autonomous(name="1RightPark", group="Tutorials")

public class detectingothernirvan extends LinearOpMode {






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
        turn.setPosition(0.7);
        claw1.setPosition(0.2);//claw close
        claw2.setPosition(0.9);
        sleep(500);
        rightslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setTargetPosition(300);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslide.setPower(1);

        leftslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);

        leftslide.setTargetPosition(300);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
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
            telemetry.addLine("Autonomous C");
            telemetry.update();
            sleep(20);
            Pose2d start = new Pose2d(40, -62, Math.toRadians(-90));
            drive.setPoseEstimate(start);

            TrajectorySequence drop_1 = drive.trajectorySequenceBuilder(start)
//                      .strafeRight(46)
//                        .splineToLinearHeading(new Pose2d(-34,-11, Math.toRadians(-180)),Math.toRadians(150))
//                        .turn(Math.toRadians(50))
                    .back(35)
                    .splineTo(new Vector2d(35,-11),Math.toRadians(150))
                    .addTemporalMarker(0.4, () -> { // from -20, -10 --> Shrish
                        upslide();
                    })
                    .addTemporalMarker(0.6, () -> { // from -20, -10 --> Shrish
                        turn.setPosition(0.9);
                    })
                    .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                        uparm();
                    })
                    .addTemporalMarker(2.4, () -> { // from -20, -10 --> Shrish
                        turno();
                    })

                    .build();




            drive.followTrajectorySequence(drop_1);
            sleep(500);
            rightslide.setTargetPosition(1900);
            leftslide.setTargetPosition(1900);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(600);
            drop();
            sleep(200);

//                drive.followTrajectorySequence(pick_1);
//                upslide();
//                drive.followTrajectory(drop_2);
//                sleep(500);

            rightslide.setTargetPosition(1900);
            leftslide.setTargetPosition(1900);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(600);
            drop();
            sleep(200);

//                drive.followTrajectorySequence(pick_2);
//
//                drive.followTrajectory(drop_3);
//                sleep(700);

            rightslide.setTargetPosition(1900);
            leftslide.setTargetPosition(1900);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(600);
            drop();
            sleep(200);

//                drive.followTrajectorySequence(pick_3);
//
//                drive.followTrajectory(drop_4);
//                sleep(500);

            rightslide.setTargetPosition(1900);
            leftslide.setTargetPosition(1900);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            drop();
            sleep(200);
//                drive.followTrajectory(pick_4);




            Trajectory pick_4 = drive.trajectoryBuilder(drop_1.end())
                    .lineToLinearHeading(new Pose2d(38,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)
                    .addTemporalMarker(0, () -> {
                        pickup();
                    })
                    .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                        turnc();
                    })
                    .addTemporalMarker(0.3, () -> {
                        arm.setTargetPosition(10);
                        arm.setPower(0.7);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(0, () -> {
                        rightslide.setTargetPosition(2100); // 760
                        leftslide.setTargetPosition(2100); // 760
                        rightslide.setPower(1);
                        leftslide.setPower(1);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(1.1, () -> {
                        // this is where it picks up for the first time from the stack
                        drop();
                    })

                    .build();
            Trajectory pick_5 = drive.trajectoryBuilder(pick_4.end(),true)
                    .lineToLinearHeading(new Pose2d(18,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)

                    .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                        turnc();
                    })
                    .addTemporalMarker(0.4, () -> {
                        arm.setTargetPosition(0);
                        arm.setPower(1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(0.6, () -> {
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
            drive.followTrajectory(pick_4);
            drive.followTrajectory(pick_5);
            return;
        }
        else
        {
            /*
             * Insert your code here, probably using the tag pose to decide your configuration.
             */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //1 dot
            if(tagOfInterest.id == 3)
            {

                telemetry.addLine("Autonomous B");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(40, -62, Math.toRadians(-90));
                drive.setPoseEstimate(start);

                TrajectorySequence drop_1 = drive.trajectorySequenceBuilder(start)
//                      .strafeRight(46)
//                        .splineToLinearHeading(new Pose2d(-34,-11, Math.toRadians(-180)),Math.toRadians(150))
//                        .turn(Math.toRadians(50))
                        .back(35)
                        .splineTo(new Vector2d(36,-8),Math.toRadians(150))
                        .addTemporalMarker(0.4, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(0.6, () -> { // from -20, -10 --> Shrish
                            turn.setPosition(0.9);
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .addTemporalMarker(2.4, () -> { // from -20, -10 --> Shrish
                            turno();
                        })

                        .build();


                drive.followTrajectorySequence(drop_1);
                sleep(500);
                rightslide.setTargetPosition(2000);
                leftslide.setTargetPosition(2000);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_1);
//                upslide();
//                drive.followTrajectory(drop_2);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_2);
//
//                drive.followTrajectory(drop_3);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_3);
//
//                drive.followTrajectory(drop_4);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drop();
                sleep(200);

                TrajectorySequence pick_4 = drive.trajectorySequenceBuilder(drop_1.end())
                        .lineToLinearHeading(new Pose2d(38,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            turnc();
                        })
                        .addTemporalMarker(0.3, () -> {
                            arm.setTargetPosition(10);
                            arm.setPower(0.7);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0, () -> {
                            rightslide.setTargetPosition(2100); // 760
                            leftslide.setTargetPosition(2100); // 760
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(1.1, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })

                        .build();


                drive.followTrajectorySequence(pick_4);
                // drive.followTrajectory(pick_5);

//

                return;

            }
            else if(tagOfInterest.id==0)
            {
                telemetry.addLine("Autonomous A");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(40, -62, Math.toRadians(-90));
                drive.setPoseEstimate(start);

                TrajectorySequence drop_1 = drive.trajectorySequenceBuilder(start)
//                      .strafeRight(46)
//                        .splineToLinearHeading(new Pose2d(-34,-11, Math.toRadians(-180)),Math.toRadians(150))
//                        .turn(Math.toRadians(50))
                        .back(35)
                        .splineTo(new Vector2d(36,-8),Math.toRadians(150))
                        .addTemporalMarker(0.4, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(0.6, () -> { // from -20, -10 --> Shrish
                            turn.setPosition(0.9);
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .addTemporalMarker(2.4, () -> { // from -20, -10 --> Shrish
                            turno();
                        })

                        .build();



                drive.followTrajectorySequence(drop_1);
                sleep(500);
                rightslide.setTargetPosition(2000);
                leftslide.setTargetPosition(2000);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_1);
//                upslide();
//                drive.followTrajectory(drop_2);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_2);
//
//                drive.followTrajectory(drop_3);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_3);
//
//                drive.followTrajectory(drop_4);
//                sleep(500);

                rightslide.setTargetPosition(2100);
                leftslide.setTargetPosition(2100);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drop();
                sleep(200);



//                drive.followTrajectorySequence(pick_4);

                TrajectorySequence pick_4 = drive.trajectorySequenceBuilder(drop_1.end())
                        .lineToLinearHeading(new Pose2d(38,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            turnc();
                        })
                        .addTemporalMarker(0.3, () -> {
                            arm.setTargetPosition(10);
                            arm.setPower(0.7);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0, () -> {
                            rightslide.setTargetPosition(2100); // 760
                            leftslide.setTargetPosition(2100); // 760
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(1.1, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })

                        .build();

                drive.followTrajectorySequence(pick_4);

//

                return;



                }
            else if(tagOfInterest.id == 6)
            {
                telemetry.addLine("Autonomous C");
                telemetry.update();
                sleep(20);
                Pose2d start = new Pose2d(40, -62, Math.toRadians(-90));
                drive.setPoseEstimate(start);

                TrajectorySequence drop_1 = drive.trajectorySequenceBuilder(start)
//                      .strafeRight(46)
//                        .splineToLinearHeading(new Pose2d(-34,-11, Math.toRadians(-180)),Math.toRadians(150))
//                        .turn(Math.toRadians(50))
                        .back(35)
                        .splineTo(new Vector2d(35,-11),Math.toRadians(150))
                        .addTemporalMarker(0.4, () -> { // from -20, -10 --> Shrish
                            upslide();
                        })
                        .addTemporalMarker(0.6, () -> { // from -20, -10 --> Shrish
                            turn.setPosition(0.9);
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            uparm();
                        })
                        .addTemporalMarker(2.4, () -> { // from -20, -10 --> Shrish
                            turno();
                        })

                        .build();




                drive.followTrajectorySequence(drop_1);
                sleep(500);
                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_1);
//                upslide();
//                drive.followTrajectory(drop_2);
//                sleep(500);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_2);
//
//                drive.followTrajectory(drop_3);
//                sleep(700);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(600);
                drop();
                sleep(200);

//                drive.followTrajectorySequence(pick_3);
//
//                drive.followTrajectory(drop_4);
//                sleep(500);

                rightslide.setTargetPosition(1900);
                leftslide.setTargetPosition(1900);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drop();
                sleep(200);
//                drive.followTrajectory(pick_4);




                Trajectory pick_4 = drive.trajectoryBuilder(drop_1.end())
                        .lineToLinearHeading(new Pose2d(38,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)
                        .addTemporalMarker(0, () -> {
                            pickup();
                        })
                        .addTemporalMarker(0.2, () -> { // from -20, -10 --> Shrish
                            turnc();
                        })
                        .addTemporalMarker(0.3, () -> {
                            arm.setTargetPosition(10);
                            arm.setPower(0.7);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0, () -> {
                            rightslide.setTargetPosition(2100); // 760
                            leftslide.setTargetPosition(2100); // 760
                            rightslide.setPower(1);
                            leftslide.setPower(1);
                            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(1.1, () -> {
                            // this is where it picks up for the first time from the stack
                            drop();
                        })

                        .build();
                Trajectory pick_5 = drive.trajectoryBuilder(pick_4.end(),true)
                        .lineToLinearHeading(new Pose2d(18,-11,Math.toRadians(-90)))
//                        .splineTo(new Vector2d(-64,-15),Math.toRadians(-170))

//                        .forward(15.5)

                        .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                            turnc();
                        })
                        .addTemporalMarker(0.4, () -> {
                            arm.setTargetPosition(0);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(0.6, () -> {
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
                drive.followTrajectory(pick_4);
                drive.followTrajectory(pick_5);

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
        rightslide.setTargetPosition(2800);
        leftslide.setTargetPosition(2800);
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
        claw1.setPosition(0.2);//claw close
        claw2.setPosition(0.9);
    }
    public void drop(){
        claw1.setPosition(0.3);//claw open
        claw2.setPosition(0.6); // 0.5

    }
    public void turno(){
        turn.setPosition(0.3);
    }
    public void turnc(){
        turn.setPosition(1);
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