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


@Autonomous(name="Backup", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class BACKUP extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor verticalLift = null;
    Servo claw = null;
    Servo claw2 = null;
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
    boolean flag = true;
    boolean flag2 = true;
    boolean flag3 = true;
    boolean flag4 = true;
    boolean flag5 = true;
    int stagecounter = 0;
    int leftFrontPos = 0;
    int motorRotation = 7100;


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
        claw = hardwareMap.servo.get("Lclaw");
        claw2 = hardwareMap.servo.get("Rclaw");
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
        claw.setDirection(Servo.Direction.FORWARD);
        claw2.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        claw.setPosition(0.84);
        claw2.setPosition(1);
        //lineSensor.enableLed(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            arm.setPosition(.05);
            verticalLift.setTargetPosition(2000);
            verticalLift.setPower(1);
            if (stage1) {
                if (color.red() > 1 || color.blue() > 1 && runtime.milliseconds() > 2000) {
                    if (color.red() > color.blue()) {
                        if (red) {
                            leftFront.setTargetPosition(300);
                            leftFront.setPower(.4);
                            rightFront.setTargetPosition(300);
                            rightFront.setPower(.4);
                            leftBack.setTargetPosition(300);
                            leftBack.setPower(.4);
                            rightBack.setTargetPosition(300);
                            rightBack.setPower(.4);
                            stage1 = false;
                            stage2 = true;
                            stagecounter ++;
                        } else {
                            leftFront.setTargetPosition(-300);
                            leftFront.setPower(.4);
                            rightFront.setTargetPosition(-300);
                            rightFront.setPower(.4);
                            leftBack.setTargetPosition(-300);
                            leftBack.setPower(.4);
                            rightBack.setTargetPosition(-300);
                            rightBack.setPower(.4);
                            stage1 = false;
                            stage2 = true;
                            stagecounter ++;
                        }
                    } else {
                        if (red) {
                            leftFront.setTargetPosition(-300);
                            leftFront.setPower(.4);
                            rightFront.setTargetPosition(-300);
                            rightFront.setPower(.4);
                            leftBack.setTargetPosition(-300);
                            leftBack.setPower(.4);
                            rightBack.setTargetPosition(-300);
                            rightBack.setPower(.4);
                            stage1 = false;
                            stage2 = true;
                            stagecounter ++;
                        } else {
                            leftFront.setTargetPosition(300);
                            leftFront.setPower(.4);
                            rightFront.setTargetPosition(300);
                            rightFront.setPower(.4);
                            leftBack.setTargetPosition(300);
                            leftBack.setPower(.4);
                            rightBack.setTargetPosition(300);
                            rightBack.setPower(.4);
                            stage1 = false;
                            stage2 = true;
                            stagecounter ++;
                        }
                    }
                }
            }

            if (stage2) {
                arm.setPosition(.5);
                if (red && Math.abs(leftFront.getCurrentPosition()) > -1) {
                    if (flag) {
                        leftFrontPos = leftFront.getCurrentPosition();
                        flag = false;
                    }
                    leftFront.setTargetPosition(leftFrontPos + 3000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(leftFrontPos + 3000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftFrontPos + 3000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(leftFrontPos + 3000);
                    rightBack.setPower(.4);
                    if(leftFront.getCurrentPosition() - leftFrontPos > 2470) {
                        stage3 = true;
                        stage2 = false;
                        stagecounter ++;
                    }
                } else if (Math.abs(leftFront.getCurrentPosition()) > -1 && flag3) {
                    if (flag) {
                        leftFrontPos = leftFront.getCurrentPosition();
                        flag = false;
                    }
                    //original = 5976
                    //turning
                    /*if(red){
                        leftFront.setTargetPosition(leftFront.getCurrentPosition() - motorRotation);
                        leftFront.setPower(.4);
                        rightFront.setTargetPosition(leftFront.getCurrentPosition() + motorRotation);
                        rightFront.setPower(.4);
                        leftBack.setTargetPosition(leftFront.getCurrentPosition() - motorRotation);
                        leftBack.setPower(.4);
                        rightBack.setTargetPosition(leftFront.getCurrentPosition() + motorRotation);
                        rightBack.setPower(.4);
                    }*/

                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + motorRotation);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() - motorRotation);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + motorRotation);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() - motorRotation);
                    rightBack.setPower(.4);

                    if (leftFront.getCurrentPosition() - leftFrontPos > 4000) {
                        if (flag2) {
                            leftFrontPos = leftFront.getCurrentPosition();
                            flag2 = false;
                            flag3 = false;
                            stagecounter ++;
                        }
                        leftFront.setTargetPosition(leftFront.getCurrentPosition() + 6700);
                        leftFront.setPower(.4);
                        rightFront.setTargetPosition(rightFront.getCurrentPosition() + 6700);
                        rightFront.setPower(.4);
                        leftBack.setTargetPosition(leftBack.getCurrentPosition() + 6700);
                        leftBack.setPower(.4);
                        rightBack.setTargetPosition(rightBack.getCurrentPosition() + 6700);
                        rightBack.setPower(.4);
                        //use 5squared pluse 6.5squared = xsquared
                        if (leftFront.getCurrentPosition() - leftFrontPos > 6650) {
                            stage3 = true;
                            stage2 = false;
                            stagecounter ++;
                        }
                    }
                }
            }
            if (stage3) {
                if(flag4) {
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag4 = false;
                    stagecounter ++;
                }
                //going to the left/right/center hole based off the initial cryptograph
                if (right) {
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 1000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 1000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 1000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 1000);
                    rightBack.setPower(.4);
                    if (leftFront.getCurrentPosition() - leftFrontPos > 470) {
                        flag4 = true;
                        stage4 = true;
                        stage3 = false;
                        stagecounter ++;
                    }
                }
                if (left) {
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 5000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 5000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 5000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 5000);
                    rightBack.setPower(.4);
                    if (leftFront.getCurrentPosition() - leftFrontPos > 1470) {
                        flag4 = true;
                        stage4 = true;
                        stage3 = false;
                        stagecounter ++;
                    }
                }
                //else is for no reading or center
                //this one reaches the far box - not the center one
                else{
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 3000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 3000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 3000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 3000);
                    rightBack.setPower(.4);
                    if (leftFront.getCurrentPosition() - leftFrontPos > 770) {
                        flag4 = true;
                        stage4 = true;
                        stage3 = false;
                        stagecounter ++;
                    }
                }
            }
            if (stage4) {
                if(flag4){
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag4 = false;
                }
                if (red) {
                    /**switch these signs for blue*/
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 800);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() - 800);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 800);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() - 800);
                    rightBack.setPower(.4);
                } else {
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() - 800);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 800);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() - 800);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 800);
                    rightBack.setPower(.4);
                }
                if(Math.abs(leftFront.getCurrentPosition() - leftFrontPos) > 1200) {
                    stage5 = true;
                    stage4 = false;
                    stagecounter ++;
                }

            }
            if(stage5){
                if(flag5) {
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag5 = false;
                }
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + 3000);
                leftFront.setPower(.4);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + 3000);
                rightFront.setPower(.4);
                leftBack.setTargetPosition(leftBack.getCurrentPosition() + 3000);
                leftBack.setPower(.4);
                rightBack.setTargetPosition(rightBack.getCurrentPosition() + 3000);
                rightBack.setPower(.4);
                if(leftFront.getCurrentPosition() - leftFrontPos > 1990) {
                    claw.setPosition(0.45);
                    claw2.setPosition(0.63);
                    stage5 = false;
                    stage6 = true;
                    stagecounter ++;
                }
            }
            if(stage6){
                if(!flag5){
                    leftFrontPos = leftFront.getCurrentPosition();
                    flag5 = true;
                }
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - 2000);
                leftFront.setPower(0.4);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - 2000);
                rightFront.setPower(0.4);
                leftBack.setTargetPosition(leftBack.getCurrentPosition() - 2000);
                leftBack.setPower(0.4);
                rightBack.setTargetPosition(rightBack.getCurrentPosition() - 2000);
                rightBack.setPower(0.4);
                if(leftFrontPos - leftFront.getCurrentPosition() > 900){
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    stage6 = false;
                    stagecounter++;
                }
            }
        }
    }
}