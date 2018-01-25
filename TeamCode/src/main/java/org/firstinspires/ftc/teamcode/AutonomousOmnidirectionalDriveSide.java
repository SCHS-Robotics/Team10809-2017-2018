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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="AutonomousOmnidirectionalDriveSide", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutonomousOmnidirectionalDriveSide extends LinearOpMode {

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

    MediaPlayer start_sound;

    //driving variables

    boolean stage1 = true;
    //color sensing
    boolean stage2 = false;
    //jewel knocking
    boolean stage3 = false;
    //LRC moving
    boolean stage4 = false;
    //turning
    boolean stage5 = false;
    //claw dropping
    boolean stage6 = false;
    //backing up
    boolean left = false;
    boolean right = false;
    boolean center = false;
    boolean red = true;
    boolean flag = false;
    int stageCounter = 0;
    int leftFrontPos = 0;
    int motorRotation = 7100;
    double MOTORPOWER = 0.35;
    double ARMUP = 0.6;
    double ARMDOWN = 0.08;
    boolean graciousProfessionalism = true;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        start_sound = MediaPlayer.create(hardwareMap.appContext, R.raw.blitzcrank_startup);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        Lclaw = hardwareMap.servo.get("Lclaw");
        Rclaw = hardwareMap.servo.get("Rclaw");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");

        //color.setI2cAddress(new I2cAddr(0x39));
        //this ^ is not actually needed anymore

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        verticalLift.setDirection(DcMotor.Direction.FORWARD);
/*        if(!red){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        }*/
        Lclaw.setDirection(Servo.Direction.FORWARD);
        Rclaw.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //VUFORIA

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ARRYVKz/////AAAAGZxuAEFNDkCrkYt707UsjihZs15F76lsvH7AU/mlPnRZ3yAdhedSbovCnzPrTc4U6nQU0BbKTmXyYv+6l4YQzmIMIos9kWdCc9mFhExHofogzzGejNg38CogHWqIUFqwvbTFIzTwvsTDFTEJuJAduMh1nl4ui9YHjRWv5I3vrBJ96kzkIO1aC23JBA9w+JsMAXKk0PyBitnXq8hTY2x4SM8IVwmRJontBEvr3BUIHi2P8E1sMznS2bEshTvwmg2nOnD6IA9ChrKIP/YVbsO1HHGm9fmqTfoN/VBOiUskbzNBcmylv0jPZOhq+X2LnMRZinss3ZWn8KQE1VLPeVSIJdEAwx8rqyX+wvkqriFVwae/";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();


        telemetry.addData("Status", "Ready to begin");
        telemetry.update();
        start_sound.start();
        waitForStart();
        runtime.reset();
        color.enableLed(true);
        Lclaw.setPosition(0.84);
        Rclaw.setPosition(1);
        arm.setPosition(ARMDOWN);
        //lineSensor.enableLed(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Gracious professionalism:", " " + graciousProfessionalism);
            telemetry.addData("Red:", " " + color.red());
            telemetry.addData("Blue:", " " + color.blue());
            telemetry.addData("Green:", " " + color.green());
            telemetry.addData("Dylan:", " is bad");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("stage1:", " " + stage1);
            telemetry.addData("stage2:", " " + stage2);
            telemetry.addData("stage3:", " " + stage3);
            telemetry.addData("stage4:", " " + stage4);
            telemetry.addData("stage5:", " " + stage5);
            telemetry.addData("stage6:", " " + stage6);
            telemetry.addData("stage changes:", " " + stageCounter);
            telemetry.addData("verticalLift:", " " + verticalLift.getCurrentPosition());
            //telemetry.addData("Color: ", color.red() + " " + color.green() + " " + color.blue());
            telemetry.update();


            //VUFORIA

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} i0s an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (!center && !left && !right) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {

                    left = true;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {

                    center = true;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                    right = true;
                } else {
                    telemetry.addData("VuMark", "not visible", vuMark);
                }
            } else if (right) {
                telemetry.addData("VuMark", "right", vuMark);
            } else if (left) {
                telemetry.addData("VuMark", "left", vuMark);
            } else if (center) {
                telemetry.addData("VuMark", "center", vuMark);
            }
            verticalLift.setTargetPosition(200);
            verticalLift.setPower(1);
            if (stage1) {
                if(!flag){
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = true;
                }
                if (color.red() > 1 || color.blue() > 1 && runtime.milliseconds() > 5000) {
                    if ((color.red() - color.blue()) > 50) {
                        if(red) {
                            leftFront.setPower(0.1);
                            rightFront.setPower(0.1);
                            leftBack.setPower(0.1);
                            rightBack.setPower(0.1);
                            if (leftFront.getCurrentPosition() - leftFrontPos > 470) {
                                stage1 = false;
                                stage2 = true;
                                stageCounter++;
                            }
                        }else{
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftBack.setPower(-0.1);
                            rightBack.setPower(-0.1);
                            if (leftFront.getCurrentPosition() - leftFrontPos < -470) {
                                stage1 = false;
                                stage2 = true;
                                stageCounter++;
                            }
                        }
                    } else {
                        if(red) {
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftBack.setPower(-0.1);
                            rightBack.setPower(-0.1);
                            if (leftFront.getCurrentPosition() - leftFrontPos < -470) {
                                stage1 = false;
                                stage2 = true;
                                stageCounter++;
                            }
                        }else{
                            leftFront.setPower(0.1);
                            rightFront.setPower(0.1);
                            leftBack.setPower(0.1);
                            rightBack.setPower(0.1);
                            if (leftFront.getCurrentPosition() - leftFrontPos > 470) {
                                stage1 = false;
                                stage2 = true;
                                stageCounter++;
                            }
                        }
                    }
                }
            }

            if (stage2) {
                arm.setPosition(ARMUP);
                if (flag) {
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = false;
                }
                if(red) {
                    leftFront.setPower(MOTORPOWER);
                    rightFront.setPower(MOTORPOWER);
                    leftBack.setPower(MOTORPOWER);
                    rightBack.setPower(MOTORPOWER);
                    if (leftFront.getCurrentPosition() > 2770) {
                        stage3 = true;
                        stage2 = false;
                        stageCounter++;
                    }
                }else{
                    leftFront.setPower(-MOTORPOWER);
                    rightFront.setPower(-MOTORPOWER);
                    leftBack.setPower(-MOTORPOWER);
                    rightBack.setPower(-MOTORPOWER);
                    if (leftFront.getCurrentPosition() < -2770) {
                        stage3 = true;
                        stage2 = false;
                        stageCounter++;
                    }
                }
            }
            if (stage3) {
                if(!flag) {
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = true;
                }
                //going to the left/right/center hole based off the initial pictograph
                if (right) {
                    if(red) {
                        leftFront.setPower(MOTORPOWER);
                        rightFront.setPower(MOTORPOWER);
                        leftBack.setPower(MOTORPOWER);
                        rightBack.setPower(MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos > 1070) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }else{
                        leftFront.setPower(-MOTORPOWER);
                        rightFront.setPower(-MOTORPOWER);
                        leftBack.setPower(-MOTORPOWER);
                        rightBack.setPower(-MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos < -1070) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }
                }
                if (left) {
                    if(red) {
                        leftFront.setPower(MOTORPOWER);
                        rightFront.setPower(MOTORPOWER);
                        leftBack.setPower(MOTORPOWER);
                        rightBack.setPower(MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos > 470) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }else{
                        leftFront.setPower(-MOTORPOWER);
                        rightFront.setPower(-MOTORPOWER);
                        leftBack.setPower(-MOTORPOWER);
                        rightBack.setPower(-MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos < -470) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }
                }
                //else is for no reading or center
                //this one reaches the far box basically every time
                else{
                    if(red) {
                        leftFront.setPower(MOTORPOWER);
                        rightFront.setPower(MOTORPOWER);
                        leftBack.setPower(MOTORPOWER);
                        rightBack.setPower(MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos > 770) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }else{
                        leftFront.setPower(-MOTORPOWER);
                        rightFront.setPower(-MOTORPOWER);
                        leftBack.setPower(-MOTORPOWER);
                        rightBack.setPower(-MOTORPOWER);
                        if (leftFront.getCurrentPosition() - leftFrontPos < -770) {
                            flag = false;
                            stage4 = true;
                            stage3 = false;
                            stageCounter++;
                        }
                    }
                }
            }
            if (stage4) {
                if(!flag){
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = true;
                }
                    leftFront.setPower(MOTORPOWER);
                    rightFront.setPower(-MOTORPOWER);
                    leftBack.setPower(MOTORPOWER);
                    rightBack.setPower(-MOTORPOWER);
                    if (Math.abs(leftFront.getCurrentPosition() - leftFrontPos) > 2000) {
                        //previoous: 1600
                        stage5 = true;
                        stage4 = false;
                        stageCounter++;
                    }

            }
            if(stage5){
                if(flag) {
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = false;
                }
                leftFront.setPower(MOTORPOWER);
                rightFront.setPower(MOTORPOWER);
                leftBack.setPower(MOTORPOWER);
                rightBack.setPower(MOTORPOWER);
                if(leftFront.getCurrentPosition() - leftFrontPos > 990) {
                    Lclaw.setPosition(0.45);
                    Rclaw.setPosition(0.63);
                    stage5 = false;
                    stage6 = true;
                    stageCounter ++;
                }
            }
            if(stage6){
                if(!flag){
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag = true;
                }
                leftFront.setPower(-MOTORPOWER);
                rightFront.setPower(-MOTORPOWER);
                leftBack.setPower(-MOTORPOWER);
                rightBack.setPower(-MOTORPOWER);
                if(leftFront.getCurrentPosition() - leftFrontPos < -900){
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    stage6 = false;
                    stageCounter++;
                }
            }
        }
    }
}