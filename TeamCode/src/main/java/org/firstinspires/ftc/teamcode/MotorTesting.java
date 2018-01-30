/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="motor testing", group="aMotorTester")  // @Autonomous(...) is the other common choice
//@Disabled
public class MotorTesting extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor verticalLift = null;
    Servo Lclaw = null;
    Servo Rclaw = null;
    Servo arm = null;
    ColorSensor color = null;

    MediaPlayer grab;
    MediaPlayer start_sound;
    MediaPlayer ult;
    MediaPlayer power_fist;
    //driving variables
    
    double deadZone = 0.15;
    double verticalLiftSpeed = 1;
    int slowFactor = 3;
    double g1Ly;
    double g1Lx;
    double g1Rx;
    int FL_motor_position;
    int FR_motor_position;
    boolean toggleA = false;
    boolean clawFlag = true;
    boolean flag2 = true;
    boolean grabFlag = false;
    boolean liftFlag = true;
    boolean toggleLB = false;

    double LclawPosition = 0.5;
    double RclawPosition = 0.5;
    double armPosition = 0.5;
    double deltaServoPostion = 0.01;







    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grab = MediaPlayer.create(hardwareMap.appContext, R.raw.rocket_grab);
        start_sound = MediaPlayer.create(hardwareMap.appContext, R.raw.blitzcrank_startup);
        ult = MediaPlayer.create(hardwareMap.appContext, R.raw.ult_sound);
        power_fist = MediaPlayer.create(hardwareMap.appContext, R.raw.power_fist);


        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        Lclaw = hardwareMap.servo.get("Lclaw");
        Rclaw = hardwareMap.servo.get("Rclaw");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        verticalLift.setDirection(DcMotor.Direction.REVERSE);
        Rclaw.setDirection(Servo.Direction.REVERSE);



        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */


        telemetry.addData("Status", "Ready to begin");
        telemetry.update();


        start_sound.start();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FL_motor_position = leftFront.getCurrentPosition();
            FR_motor_position = rightFront.getCurrentPosition();
            g1Ly = -gamepad1.left_stick_y;
            g1Lx = gamepad1.left_stick_x;
            g1Rx = gamepad1.right_stick_x;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("left trigger", " " + gamepad1.left_trigger);
            telemetry.addData("Lclaw position", " " + LclawPosition);
            telemetry.addData("Rclaw position", " " + RclawPosition);
            telemetry.addData("arm position", " " + armPosition);
            telemetry.addData("encoder position LF", " " + FL_motor_position);
            telemetry.addData("encoder position RF", " " + FR_motor_position);
            telemetry.addData("encoder position LB", " " + leftBack.getCurrentPosition());
            telemetry.addData("encoder position RB", " " + rightBack.getCurrentPosition());
            telemetry.addData("left bumper:", " " + gamepad1.left_bumper);


            telemetry.addData("Color: ", color.red() + " " + color.green() + " " + color.blue());
            telemetry.update();

            if(gamepad1.dpad_up){
                leftFront.setPower(0.5);
            } else {
                leftFront.setPower(0);
            }

            if (gamepad1.dpad_left){
                leftBack.setPower(0.5);
            } else {
                leftBack.setPower(0);
            }

            if (gamepad1.dpad_down){
                rightBack.setPower(0.5);
            } else {
                rightBack.setPower(0);
            }

            if (gamepad1.dpad_right){
                rightFront.setPower(0.5);
            } else {
                rightBack.setPower(0);
            }


            if (gamepad1.x){
                LclawPosition += deltaServoPostion;
            } else if(gamepad1.a){
                LclawPosition -= deltaServoPostion;
            }
            Lclaw.setPosition(LclawPosition);

            if (gamepad1.y){
                RclawPosition += deltaServoPostion;
            } else if (gamepad1.b){
                RclawPosition -= deltaServoPostion;
            }
            Rclaw.setPosition(RclawPosition);

            if (gamepad1.right_bumper){
                armPosition += deltaServoPostion;
            } else if (gamepad1.left_bumper){
                armPosition -= deltaServoPostion;
            }
            arm.setPosition(armPosition);
            telemetry.addData("red" , " " + color.red());
            telemetry.addData("blue" , " " + color.blue());
            telemetry.addData("green" , " " +color.green());
            

        }
    }
}
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM
//GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM GRACIOUS PROFESSIONALISM