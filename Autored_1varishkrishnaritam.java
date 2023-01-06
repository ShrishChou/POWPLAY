/*
This case is where we drop off an element, do the duck wheel, then go and park along the wall
(Except on the blue side instead of red side this time.)
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Autonomous(name="Autored_1varishkrishnaritam", group="Tutorials")

public class Autored_1varishkrishnaritam extends LinearOpMode {


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

    DcMotorEx leftFront = null;
    DcMotorEx rightRear = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftRear = null;
    CRServo duck = null;
//    Servo flip2 = null;
//    Servo stopper = null;
    DcMotor slides = null;
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
        waitForStart();
        while(opModeIsActive()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

            //        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
            //        linearslideleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //        linearslideleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //        linearslideleft.setTargetPosition(0);
            //        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//         .addTemporalMarker(0, () -> {
//                    x();
//                })

            Pose2d start = new Pose2d(-36, -62, Math.toRadians(-90));
            drive.setPoseEstimate(start);
            Trajectory initial_strafe = drive.trajectoryBuilder(start)
                    .strafeLeft(24)
                    .build();
            Trajectory move_to_first_deposit = drive.trajectoryBuilder(initial_strafe.end(),true)
                    .lineToLinearHeading(new Pose2d(-16, -14,Math.toRadians(150)))
                    .build();
            Trajectory transition_to_cycle = drive.trajectoryBuilder(move_to_first_deposit.end())
//                    .splineTo(new Vector2d(-60, -8), Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(-60, -12,Math.toRadians(180)))

                    .build();
            Trajectory second_drop = drive.trajectoryBuilder(transition_to_cycle.end(),true)
                    .lineToLinearHeading(new Pose2d(-16, -8,Math.toRadians(150)))
//                    .splineTo(new Vector2d(-16, -5), Math.toRadians(210))
                    .build();
            Trajectory second_pickup = drive.trajectoryBuilder(second_drop.end())
                    .lineToLinearHeading(new Pose2d(-60, -14,Math.toRadians(180)))
//                    .splineTo(new Vector2d(-60, -4), Math.toRadians(180))
                    .build();
            Trajectory third_drop = drive.trajectoryBuilder(second_pickup.end(),true)
                    .lineToLinearHeading(new Pose2d(-10, -8,Math.toRadians(150)))
                    .build();
            drive.followTrajectory(initial_strafe);
            drive.followTrajectory(move_to_first_deposit);
            drive.followTrajectory(transition_to_cycle);
            drive.followTrajectory(second_drop);
            drive.followTrajectory(second_pickup);
            drive.followTrajectory(third_drop);

            return;
//        drive.followTrajectorySequence(dk);


        }
    }
}