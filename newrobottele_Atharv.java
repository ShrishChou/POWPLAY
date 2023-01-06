/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="newrobotteleAtharv", group="Linear Opmode")

public class newrobottele_Atharv extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx arm = null;

    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightslide = null;
    private DcMotorEx leftslide = null;
    private Servo claw = null;
    private Servo stop = null;
    private Servo twist = null;


    // @Override


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftslide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        claw = hardwareMap.get(Servo.class, "claw");
        stop = hardwareMap.get(Servo.class, "stop");

        twist = hardwareMap.get(Servo.class, "twist");


//        extend = hardwareMap.get(Servo.class, "");



        boolean time = true;
        int height = 3;
        boolean turn_servo = false;
        boolean open_claw = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftslide.setDirection(DcMotorEx.Direction.REVERSE);
        rightslide.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightslide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setTargetPosition(0);
        rightslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        leftslide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftslide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //leftslide.setDirection(DcMotorEx.Direction.REVERSE);
        leftslide.setTargetPosition(0);
        leftslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


//        turn.setPosition(1); // shuold be 1
//        claw1.setPosition(0.3);//claw open
//        claw2.setPosition(0.6);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean up = false;
        double claw_pos;
        boolean turned = false;
        double y;
        double x;
        double rx;
        while (opModeIsActive()) {
//            FtcDashboard dashboard = FtcDashboard.getInstance();
//            telemetry = dashboard.getTelemetry();

            if (gamepad1.right_trigger != 0) {
                y = -gamepad1.left_stick_y; // Remember, this is reversed!
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x * 0.8;
            } else {
                y = -gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
                x = gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x * 0.5;
            }

            int counteropen = 0;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////gamepad2

            leftFront.setPower(frontLeftPower);
            rightRear.setPower(backRightPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            // turn.setPosition(turnServoPower);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////gamepad1
//            if(gamepad1.dpad_up){ // linear slide
//                rightslide.setTargetPosition(rightslide.getCurrentPosition()+130);
//                leftslide.setTargetPosition(leftslide.getCurrentPosition()+130);
//                rightslide.setPower(1);
//                leftslide.setPower(1);
//                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if(gamepad1.dpad_down){
//                rightslide.setTargetPosition(rightslide.getCurrentPosition()-130);
//                leftslide.setTargetPosition(leftslide.getCurrentPosition()-130);
//                rightslide.setPower(1);
//                leftslide.setPower(1);
//                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if(gamepad1.dpad_left){ // linear slide
//                height=1;
//                arm.setTargetPosition(-307);
//                arm.setPower(0.75);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightslide.setTargetPosition(0);
//                leftslide.setTargetPosition(0);
//                rightslide.setPower(1);
//                leftslide.setPower(1);
//                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(arm.getCurrentPosition()<=-260){
//                    turn.setPosition(0.3);
//                }
//
//            }
//            else if(gamepad1.dpad_right){
//                height=2;
//                arm.setTargetPosition(-307);
//                arm.setPower(0.75);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightslide.setTargetPosition(1200);
//                leftslide.setTargetPosition(1200);
//                rightslide.setPower(1);
//                leftslide.setPower(1);
//                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(arm.getCurrentPosition()<=-260){
//                    turn.setPosition(0.3);
//                }
//
//            }
//
            if (gamepad1.dpad_up) {
                rightslide.setTargetPosition(rightslide.getCurrentPosition()+200);
                leftslide.setTargetPosition(leftslide.getCurrentPosition()+200);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad1.dpad_down) {
                rightslide.setTargetPosition(rightslide.getCurrentPosition()-200);
                leftslide.setTargetPosition(leftslide.getCurrentPosition()-200);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (gamepad1.x) {
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(0);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(0);
                leftslide.setTargetPosition(0);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else if (gamepad1.y) {
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(0);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(1200);
                leftslide.setTargetPosition(1200);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.a) {
                claw.setPosition(0.455);//close claw

            }
            else if (gamepad1.b) {
                claw.setPosition(0.2);//open claw
            }
            if(gamepad1.left_bumper){
//                twist.setPosition(0.97);//upright
//                leftarm.setPosition(1);
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(0);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(2600);
                leftslide.setTargetPosition(2600);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            else if(gamepad1.right_bumper){
                twist.setPosition(0.94);//pickup
                arm.setTargetPosition(2125);
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(0);
                leftslide.setTargetPosition(0);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stop.setPosition(0.65);//hold poll
            }


            if (gamepad2.dpad_up) {
                rightslide.setTargetPosition(rightslide.getCurrentPosition()+200);
                leftslide.setTargetPosition(leftslide.getCurrentPosition()+200);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_down) {
                rightslide.setTargetPosition(rightslide.getCurrentPosition()-200);
                leftslide.setTargetPosition(leftslide.getCurrentPosition()-200);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (gamepad2.x) {
                stop.setPosition(0.65);//hold poll
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(577);
                arm.setPower(0.5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(0);
                leftslide.setTargetPosition(0);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else if (gamepad2.y) {
                stop.setPosition(0.65);//hold poll
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(577);
                arm.setPower(0.5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(1300);
                leftslide.setTargetPosition(1300);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if (gamepad2.a) {
                claw.setPosition(0.455);//close claw

            }
            else if (gamepad2.b) {
                claw.setPosition(0.2);//open claw


            }
            if(gamepad2.left_bumper){
//                twist.setPosition(0.97);//upright
//                leftarm.setPosition(1);
                stop.setPosition(0.65);//hold poll
                twist.setPosition(0.28);//dropfacing
                arm.setTargetPosition(577);
                arm.setPower(0.5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(2400);
                leftslide.setTargetPosition(2400);
                rightslide.setPower(1);
                leftslide.setPower(-1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            else if(gamepad2.right_bumper){
                twist.setPosition(0.94);//pickup
                arm.setTargetPosition(0);
                arm.setPower(0.5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightslide.setTargetPosition(0);
                leftslide.setTargetPosition(0);
                rightslide.setPower(1);
                leftslide.setPower(1);
                rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stop.setPosition(0.65);//hold poll

            }
            else if(gamepad1.dpad_left)
            {

            }
            else if(gamepad1.dpad_right)
            telemetry.addData("leftslidepos", leftslide.getCurrentPosition());
            telemetry.addData("rightslidepos", rightslide.getCurrentPosition());
            telemetry.addData("armpos", arm.getCurrentPosition());

            telemetry.update();

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////bools

        }
    }
}