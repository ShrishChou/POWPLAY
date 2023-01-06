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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Config
@Autonomous(name="Autoblue", group="Tutorials")

public class Autoblue_1 extends LinearOpMode {


    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution 360

    double CrLowerUpdate = 0;
    double CbLowerUpdate = 0;
    double CrUpperUpdate = 150;
    double CbUpperUpdate = 150;
    ColorSensor color = null;
    DistanceSensor dist = null;
    double lowerruntime = 0;
    double upperruntime = 0;
    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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


    // blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0, 50);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120, 130);

    @Override
    public void runOpMode() {
        //working?
        /*
            Trajectory initial_strafe = drive.trajectoryBuilder(start)
                    .strafeLeft(20)
                    .build();
            Trajectory move_to_first_deposit = drive.trajectoryBuilder(initial_strafe.end(),true)
                    .lineToLinearHeading(new Pose2d(-16, -14,Math.toRadians(135)))
                    .build();
            Trajectory transition_to_cycle = drive.trajectoryBuilder(move_to_first_deposit.end())
                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                    .build();
         */
        telemetry.addLine("Autonomous A");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm  = hardwareMap.get(DcMotorEx.class, "leftarm");
//        rightarm  = hardwareMap.get(DcMotorEx.class, "leftarm");
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
        claw2.setPosition(0.85);
        sleep(300);
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
        waitForStart();


        while(opModeIsActive()) {





            //        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
            //        linearslideleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //        linearslideleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //        linearslideleft.setTargetPosition(0);
            //        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            Pose2d start = new Pose2d(-40, -62, Math.toRadians(-180));
            drive.setPoseEstimate(start);
            Trajectory initial_strafe = drive.trajectoryBuilder(start,true)
                    .splineTo(new Vector2d(-14,-56),Math.toRadians(90))
//                    .forward(30)
//                    .splineTo(new Vector2d(-14,-56),Math.toRadians(-180))
                    .build();
//            Trajectory miss = drive.trajectoryBuilder(initial_strafe.end())
////                    .forward(10)
//                    .lineTo(new Vector2d(-17,-50))
//                    .build();

            Trajectory move_to_first_deposit = drive.trajectoryBuilder(initial_strafe.end(),true)
//                    .forward(10)
                    .lineToLinearHeading(new Pose2d(-16, -12,Math.toRadians(150))) //-14
                    .addTemporalMarker(3, () -> { // from -20, -10 --> Shrish
                        drop();
                    })
                    .build();

            Trajectory transition_to_cycle = drive.trajectoryBuilder(move_to_first_deposit.end())
//                    .splineTo(new Vector2d(-60, -8), Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(-67, -8,Math.toRadians(180)))
                    .addTemporalMarker(0, () -> {
                        pickup();
                    })
                    .addTemporalMarker(0.1, () -> {
                        rightslide.setTargetPosition(600); // 550
                        leftslide.setTargetPosition(600); // 550
                        rightslide.setPower(0.75);
                        leftslide.setPower(0.75);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(0);
                        arm.setPower(0.1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(1.7, () -> {
                        // this is where it picks up for the first time from the stack
                        drop();
                    })
                    .addTemporalMarker(2.5, () -> {
                        pickup();
                    })
                    .addTemporalMarker(3, () -> {
                        rightslide.setTargetPosition(1000);
                        leftslide.setTargetPosition(1000);
                        rightslide.setPower(0.75);
                        leftslide.setPower(0.75);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(0);
                        arm.setPower(0.1);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    })
                    .build();
            Trajectory second_drop = drive.trajectoryBuilder(transition_to_cycle.end(),true)
                    .lineToLinearHeading(new Pose2d(-8, -11,Math.toRadians(140)))
                    .addTemporalMarker(0, () -> {
                        uparm();
                    })
                    .addTemporalMarker(0, () -> {
                        upslide();
                        // turn.setPosition(0.9);//claw face front


                    })
                    .addTemporalMarker(3, () -> {
                        drop();
                        // turn.setPosition(0.2);
                    })
                    .addTemporalMarker(3.5, () -> {
                        pickup();
                    })
                    .addTemporalMarker(3.5, () -> {
                        arm.setTargetPosition(0);
                        arm.setPower(0.2);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightslide.setTargetPosition(350); // was 400
                        leftslide.setTargetPosition(350);
                        rightslide.setPower(0.75);
                        leftslide.setPower(0.75);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    })

//                    .splineTo(new Vector2d(-16, -5), Math.toRadians(210))
                    .build();
            Trajectory second_pickup = drive.trajectoryBuilder(second_drop.end())
                    .lineToLinearHeading(new Pose2d(-66, -10,Math.toRadians(180)))

                    .addTemporalMarker(2, () -> {
                        drop();
                    })
                    .addTemporalMarker(2.3  , () -> {
                        pickup();
                    })
//                    .splineTo(new Vector2d(-60, -4), Math.toRadians(180))
                    .build();
            Trajectory third_drop = drive.trajectoryBuilder(second_pickup.end(),true)
                    .lineToLinearHeading(new Pose2d(-9, -11,Math.toRadians(140)))
                    .addTemporalMarker(0, () -> {
                        uparm();
                    })
                    .addTemporalMarker(0, () -> {
                        upslide();
                    })
                    .addTemporalMarker(3, () -> {
                        drop();
                    })
                    .build();

            Trajectory third_pickup = drive.trajectoryBuilder(third_drop.end())
                    .lineToLinearHeading(new Pose2d(-66, -10,Math.toRadians(180)))

                    .addTemporalMarker(2, () -> {
                        drop();
                    })
                    .addTemporalMarker(2.3  , () -> {
                        pickup();
                    })
//                    .splineTo(new Vector2d(-60, -4), Math.toRadians(180))
                    .build();

            Trajectory fourth_drop = drive.trajectoryBuilder(third_pickup.end(),true)
                    .lineToLinearHeading(new Pose2d(-9, -11,Math.toRadians(140)))
                    .addTemporalMarker(0, () -> {
                        uparm();
                    })
                    .addTemporalMarker(0, () -> {
                        upslide();
                    })
                    .addTemporalMarker(3, () -> {
                        drop();
                    })
                    .build();

            Trajectory fourth_pickup = drive.trajectoryBuilder(fourth_drop.end())
                    .lineToLinearHeading(new Pose2d(-66, -10,Math.toRadians(180)))

                    .addTemporalMarker(2, () -> {
                        drop();
                    })
                    .addTemporalMarker(2.3  , () -> {
                        pickup();
                    })
//                    .splineTo(new Vector2d(-60, -4), Math.toRadians(180))
                    .build();

            Trajectory fifth_drop = drive.trajectoryBuilder(fourth_pickup.end(),true)
                    .lineToLinearHeading(new Pose2d(-9, -11,Math.toRadians(140)))
                    .addTemporalMarker(0, () -> {
                        uparm();
                    })
                    .addTemporalMarker(0, () -> {
                        upslide();
                    })
                    .addTemporalMarker(3, () -> {
                        drop();
                    })
                    .build();

            Trajectory fifth_pickup = drive.trajectoryBuilder(fifth_drop.end())
                    .lineToLinearHeading(new Pose2d(-66, -10,Math.toRadians(180)))
                    .addTemporalMarker(2, () -> {
                        drop();
                    })
                    .addTemporalMarker(2.3  , () -> {
                        pickup();
                    })
//                    .splineTo(new Vector2d(-60, -4), Math.toRadians(180))
                    .build();

            Trajectory sixth_drop = drive.trajectoryBuilder(fifth_pickup.end(),true)
                    .lineToLinearHeading(new Pose2d(-9, -11,Math.toRadians(140)))
                    .addTemporalMarker(0, () -> {
                        uparm();
                    })
                    .addTemporalMarker(0, () -> {
                        upslide();
                    })
                    .addTemporalMarker(3, () -> {
                        drop();
                    })
                    .build();

            Trajectory park = drive.trajectoryBuilder(sixth_drop.end(), true)
                    .strafeLeft(50)
//                    .forward(41.22)
                    .build();



            drive.followTrajectory(initial_strafe);
            uparm();
            sleep(700);
            upslide();
            sleep(500);
//            drive.followTrajectory(miss);
            drive.followTrajectory(move_to_first_deposit);
            drive.followTrajectory(transition_to_cycle);
            drive.followTrajectory(second_drop);
            drive.followTrajectory(second_pickup);
            drive.followTrajectory(third_drop);
            drive.followTrajectory(third_pickup);
            drive.followTrajectory(fourth_drop);
            drive.followTrajectory(fourth_pickup);
            drive.followTrajectory(fifth_drop);
            drive.followTrajectory(fifth_pickup);
            drive.followTrajectory(sixth_drop);
            drive.followTrajectory(park);
            return;
//        drive.followTrajectorySequence(dk);


        }


    }
    public void upslide(){
        rightslide.setTargetPosition(2400);
        leftslide.setTargetPosition(2400);
        rightslide.setPower(1);
        leftslide.setPower(1);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void uparm(){
        arm.setTargetPosition(-300);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void down(){

        rightslide.setTargetPosition(100);
        leftslide.setTargetPosition(100);
        rightslide.setPower(0.75);
        leftslide.setPower(0.75);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(0.1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void pickup(){
        claw1.setPosition(0.1);//claw close
        claw2.setPosition(0.85);
    }
    public void drop(){
        claw1.setPosition(0.3);//claw open
        claw2.setPosition(0.6); // 0.5
    }
}