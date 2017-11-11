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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


@TeleOp(name="BetterOmnidirectionalDrive", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class BetterOmnidirectionalDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor verticalLift = null;
    Servo claw = null;


    //driving variables
    
    double deadZone = 0.15;
    double verticalLiftSpeed = 1;
    double g1Ly;
    double g1Lx;
    double g1Rx;
    boolean left = false;
    boolean right = false;
    boolean center = false;
    boolean toggleA = false;
    boolean flag = true;


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


        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        claw = hardwareMap.servo.get("claw");
        //color = hardwareMap.colorSensor.get("color");
        //color.setI2cAddress(I2cAddr.create8bit(0x4c));


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        verticalLift.setDirection(DcMotor.Direction.FORWARD);


        //VUFORIA

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

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

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            g1Ly = -gamepad1.left_stick_y;
            g1Lx = gamepad1.left_stick_x;
            g1Rx = gamepad1.right_stick_x;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X:", " " + g1Lx);
            telemetry.addData("Y:", " " + g1Ly);
            telemetry.addData("RX:", " " + g1Rx);
            telemetry.addData("encoder position" + " ", leftFront.getCurrentPosition());
            //telemetry.addData("Color: ", color.red() + " " + color.green() + " " + color.blue());
            telemetry.update();

            //for rotation
            if (g1Rx > deadZone || g1Rx < -deadZone) {
                leftBack.setPower(g1Rx);
                rightFront.setPower(-g1Rx);
                leftFront.setPower(g1Rx);
                rightBack.setPower(-g1Rx);
            }

            //for left and right
            else if (Math.abs(g1Lx) > deadZone && Math.abs(g1Ly) < deadZone) {
                leftBack.setPower(-g1Lx);
                rightFront.setPower(-g1Lx);
                leftFront.setPower(g1Lx);
                rightBack.setPower(g1Lx);
            }

            //for up and down
            else if (Math.abs(g1Ly) > deadZone && Math.abs(g1Lx) < deadZone) {
                leftBack.setPower(g1Ly);
                rightFront.setPower(g1Ly);
                leftFront.setPower(g1Ly);
                rightBack.setPower(g1Ly);
            }

            //for top left and bottem right
            else if (g1Ly > deadZone && g1Lx < -deadZone || g1Ly < -deadZone && g1Lx > deadZone) {
                leftBack.setPower(((g1Ly - g1Lx) / 2) * 1.4);
                rightFront.setPower(((g1Ly - g1Lx) / 2) * 1.4);
                leftFront.setPower(0);
                rightBack.setPower(0);
            }

            //for top right and bottem left
            else if (g1Ly > deadZone && g1Lx > deadZone || g1Ly < -deadZone && g1Lx < -deadZone) {
                leftBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(((g1Ly + g1Lx) / 2) * 1.5);
                rightBack.setPower(((g1Ly + g1Lx) / 2) * 1.5);
            }

            //to stop
            else {
                leftBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
            }

            //for claw movement
            if (gamepad1.a && flag){
                toggleA = !toggleA;
                flag = false;
            }
            else if(!gamepad1.a) {
                flag = true;
            }
            if(!toggleA) {
                claw.setPosition(0.4);
            }
            else{
                claw.setPosition(0.6);
            }

            //for lift movement
            if(gamepad1.dpad_up)
            {
                verticalLift.setPower(verticalLiftSpeed);
            }
            else if(gamepad1.dpad_down)
            {
                verticalLift.setPower(-verticalLiftSpeed);
            }
            else
            {
                verticalLift.setPower(0);
            }

            //VUFORIA


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (!center && !left && !right)
            {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark", "left", vuMark);
                    left = true;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark", "center", vuMark);
                    center = true;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark", "right", vuMark);
                    right = true;
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
            else if(right)
            {
                telemetry.addData("VuMark", "center", vuMark);
            }

            /*if(gamepad1.a && flag)
            {
                toggleA = !toggleA;
                flag = false;
            }
            else if (!gamepad1.a)
            {
                flag = tue;
            }*/

        }
    }
}