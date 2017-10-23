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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="BetterOmnidirectionalDrive", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class BetterOmnidirectionalDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;


    //driving variables
    
    double deadZone = 0.15;
    double g1Ly;
    double g1Lx;
    double g1Rx;
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        //color = hardwareMap.colorSensor.get("color");
        //color.setI2cAddress(I2cAddr.create8bit(0x4c));


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Ready to begin");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            g1Ly = -gamepad1.left_stick_y;
            g1Lx = gamepad1.left_stick_X;
            g1Rx = gamepad1.right_stick_x;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X:", " " + g1Lx);
            telemetry.addData("Y:", " " + g1Ly);
            telemetry.addData("RX:", " " + g1Rx);
            //telemetry.addData("Color: ", color.red() + " " + color.green() + " " + color.blue());
            telemetry.update();

            //for rotation
            if (g1Rx > deadZone || -g1Rx < -deadZone)
            {
                leftBack.setPower(g1Rx);
                rightFront.setPower(-g1Rx);
                leftFront.setPower(g1Rx);
                rightBack.setPower(-g1Rx);
            }

            //for left and right
            else if (Math.abs(g1Lx) > deadZone && Math.abs(g1Ly) < deadZone)
            {
                leftBack.setPower(-g1Lx);
                rightFront.setPower(-g1Lx);
                leftFront.setPower(g1Lx);
                rightBack.setPower(g1Lx);
            }

            //for up and down
            else if (Math.abs(g1Ly) > deadZone && Math.abs(g1Lx) < deadZone)
            {
                leftBack.setPower(g1Ly);
                rightFront.setPower(g1Ly);
                leftFront.setPower(g1Ly);
                rightBack.setPower(g1Ly);
            }

            //for top left and bottem right
            else if (g1Ly > deadZone && -g1Lx < -deadZone || -g1Ly > -deadZone && g1Lx < deadZone)
            {
                leftBack.setPower(g1Ly);
            }

            

        }
    }
}