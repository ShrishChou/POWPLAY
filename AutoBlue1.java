/*
This case is where we drop off an element, do the duck wheel, then go and park along the wall
(Except on the blue side instead of red side this time.)
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Autonomous(name="BlueTank", group="Tutorials")

public class AutoBlue1 extends LinearOpMode {


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

            TrajectorySequence move_to_first_deposit = drive.trajectorySequenceBuilder(start)
//                    .forward(10)
                    .back(22.5) //-14
                    .strafeRight(49)
                    .addTemporalMarker(1.6, () -> { // from -20, -10 --> Shrish
                        upslide();
                    })

                    .build();

            TrajectorySequence transition_to_cycle = drive.trajectorySequenceBuilder(move_to_first_deposit.end())
//                    .splineTo(new Vector2d(-60, -8), Math.toRadians(180))
                    .strafeRight(8)
                    .forward(47)
                    .addTemporalMarker(0, () -> {
                        pickup();
                    })
                    .addTemporalMarker(0, () -> {
                        arm.setTargetPosition(0);
                        arm.setPower(0.8);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(0.1, () -> {
                        rightslide.setTargetPosition(600); // 550
                        leftslide.setTargetPosition(600); // 550
                        rightslide.setPower(1);
                        leftslide.setPower(1);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(1.6, () -> {
                        // this is where it picks up for the first time from the stack
                        drop();
                    })
                    .addTemporalMarker(2.8, () -> {
                        pickup();
                    })
//                    .addTemporalMarker(4.2, () -> {
//                        rightslide.setTargetPosition(1000);
//                        leftslide.setTargetPosition(1000);
//                        rightslide.setPower(0.75);
//                        leftslide.setPower(0.75);
//                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        arm.setTargetPosition(0);
//                        arm.setPower(0.1);
//                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    })
                    .build();
            TrajectorySequence second_drop = drive.trajectorySequenceBuilder(transition_to_cycle.end())
                    .back(52) //-14
                    .strafeLeft(22)
                    .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                        upslide();
                    })
//                    .splineTo(new Vector2d(-16, -5), Math.toRadians(210))
                    .build();
            TrajectorySequence transition_to_cycle_2 = drive.trajectorySequenceBuilder(second_drop.end())
//                    .splineTo(new Vector2d(-60, -8), Math.toRadians(180))
                    .strafeRight(18)
                    .forward(48)
                    .addTemporalMarker(0, () -> {
                        pickup();
                    })
                    .addTemporalMarker(0, () -> {
                        arm.setTargetPosition(0);
                        arm.setPower(0.8);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(0.1, () -> {
                        rightslide.setTargetPosition(200); // 550
                        leftslide.setTargetPosition(200); // 550
                        rightslide.setPower(1);
                        leftslide.setPower(1);
                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addTemporalMarker(1.8, () -> {
                        // this is where it picks up for the first time from the stack
                        drop();
                    })
                    .addTemporalMarker(3.3, () -> {
                        pickup();
                    })
//                    .addTemporalMarker(4.2, () -> {
//                        rightslide.setTargetPosition(1000);
//                        leftslide.setTargetPosition(1000);
//                        rightslide.setPower(0.75);
//                        leftslide.setPower(0.75);
//                        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        arm.setTargetPosition(0);
//                        arm.setPower(0.1);
//                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    })
                    .build();
            TrajectorySequence second_drop_2 = drive.trajectorySequenceBuilder(transition_to_cycle_2.end())
                    .back(49.3) //-14
                    .strafeLeft(20)
                    .addTemporalMarker(0, () -> { // from -20, -10 --> Shrish
                        upslide();
                    })
//                    .splineTo(new Vector2d(-16, -5), Math.toRadians(210))
                    .build();


            drive.followTrajectorySequence(move_to_first_deposit);
            uparm();
            sleep(2000);
            rightslide.setTargetPosition(2100);
            leftslide.setTargetPosition(2100);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            drop();
            sleep(300);


            drive.followTrajectorySequence(transition_to_cycle);
            rightslide.setTargetPosition(1000);
            leftslide.setTargetPosition(1000);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.followTrajectorySequence(second_drop);
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


            drive.followTrajectorySequence(transition_to_cycle_2);
            rightslide.setTargetPosition(1000);
            leftslide.setTargetPosition(1000);
            rightslide.setPower(1);
            leftslide.setPower(1);
            rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.followTrajectorySequence(second_drop_2);
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
//            drive.followTrajectory(second_pickup);
//            drive.followTrajectory(third_drop);
//            drive.followTrajectory(third_pickup);
//            drive.followTrajectory(fourth_drop);
//            drive.followTrajectory(fourth_pickup);
//            drive.followTrajectory(fifth_drop);
//            drive.followTrajectory(fifth_pickup);
//            drive.followTrajectory(sixth_drop);
//            drive.followTrajectory(park);
            return;
//        drive.followTrajectorySequence(dk);


        }


    }
    public void upslide(){
        rightslide.setTargetPosition(2300);
        leftslide.setTargetPosition(2300);
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
        claw1.setPosition(0.05);//claw close
        claw2.setPosition(0.9);
    }
    public void drop(){
        claw1.setPosition(0.3);//claw open
        claw2.setPosition(0.6); // 0.5
    }
}