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


@TeleOp(name="AutonomousOmnidirectionalDriveSide", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutonomousOmnidirectionalDriveSide extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor verticalLift = null;
    Servo claw = null;
    Servo arm = null;
    ColorSensor color = null;

    MediaPlayer start_sound;

    //driving variables

    boolean stage1 = false;
    boolean left = false;
    boolean right = false;
    boolean center = false;
    boolean red = false;
    boolean stage2 = true;
    boolean stage3 = false;
    boolean flag = true;
    double leftFrontPos = 0;
    boolean flag2 = true;
    boolean flag3 = true;
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
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");

        color.setI2cAddress(new I2cAddr(0x39));

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        verticalLift.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
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

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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
        //lineSensor.enableLed(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            telemetry.addData("Red:", " " + color.red());
            telemetry.addData("Blue:", " " + color.blue());
            telemetry.addData("Greem:", " " + color.green());
            telemetry.addData("Status", "Run Time: " + runtime.toString());

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
            if (!center && !left && !right)
            {
                if (vuMark == RelicRecoveryVuMark.LEFT) {

                    left = true;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {

                    center = true;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                    right = true;
                } else {
                    telemetry.addData("VuMark", "not visible", vuMark);
                }
            }
            else if(right)
            {
                telemetry.addData("VuMark", "right", vuMark);
            }
            else if(left)
            {
                telemetry.addData("VuMark", "left", vuMark);
            }
            else if(center)
            {
                telemetry.addData("VuMark", "center", vuMark);
            }
            arm.setPosition(.05);
            verticalLift.setTargetPosition(2000);
            verticalLift.setPower(1);
            if(stage1)
            {

                 if(color.red() > 1 || color.blue() > 1 && runtime.milliseconds() > 2000){
                     if(color.red() > color.blue()){
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
                         }
                         else {
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
                         }
                     }


                     else{
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
                         }
                         else{
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
                         }

                     }
                 }



            }

            if (stage2) {
                arm.setPosition(.5);
                if (red && Math.abs(leftFront.getCurrentPosition()) > -1) {
                    leftFront.setTargetPosition(2000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(2000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(2000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(2000);
                    rightBack.setPower(.4);
                    stage3 = true;
                    stage2 = false;
                } else if (Math.abs(leftFront.getCurrentPosition()) > -1 && flag3) {
                    if (flag) {
                        leftFrontPos = leftFront.getCurrentPosition();
                        flag = false;
                    }
                    //original = 5976
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + motorRotation);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(leftFront.getCurrentPosition() - motorRotation);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftFront.getCurrentPosition() + motorRotation);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(leftFront.getCurrentPosition() - motorRotation);
                    rightBack.setPower(.4);

                    if (leftFront.getCurrentPosition() - leftFrontPos > 5920) {
                        if (flag2) {
                            leftFrontPos = leftFront.getCurrentPosition();
                            flag2 = false;
                            flag3 = false;
                        }
                        leftFront.setTargetPosition(leftFront.getCurrentPosition() + 2000);
                        leftFront.setPower(.4);
                        rightFront.setTargetPosition(rightFront.getCurrentPosition() + 2000);
                        rightFront.setPower(.4);
                        leftBack.setTargetPosition(leftBack.getCurrentPosition() + 2000);
                        leftBack.setPower(.4);
                        rightBack.setTargetPosition(rightBack.getCurrentPosition() + 2000);
                        rightBack.setPower(.4);
                        //use 5squared pluse 6.5squared = xsquared
                        if (leftFront.getCurrentPosition() - leftFrontPos > 1980) {
                            stage3 = true;
                            stage2 = false;
                        }
                    }

                }
            }
            if (stage3){
                if(right){
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 1000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 1000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 1000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 1000);
                    rightBack.setPower(.4);
                }
                if (center){
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 3000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 3000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 3000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 3000);
                    rightBack.setPower(.4);
                }
                if (left){
                    leftFront.setTargetPosition(leftFront.getCurrentPosition() + 5000);
                    leftFront.setPower(.4);
                    rightFront.setTargetPosition(rightFront.getCurrentPosition() + 5000);
                    rightFront.setPower(.4);
                    leftBack.setTargetPosition(leftBack.getCurrentPosition() + 5000);
                    leftBack.setPower(.4);
                    rightBack.setTargetPosition(rightBack.getCurrentPosition() + 5000);
                    rightBack.setPower(.4);
                }
            }

        }


     }
}


