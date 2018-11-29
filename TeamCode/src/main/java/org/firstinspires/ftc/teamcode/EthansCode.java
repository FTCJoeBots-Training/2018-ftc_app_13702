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

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueReader;

/**
 * Sample code to test mapping of one motor to the gamepad.
 */
@TeleOp(name = "Concept: TeleOp Andrew this is the one pick it", group = "Concept")
//@Disabled
public class EthansCode extends LinearOpMode {


    // Define class members





    Servo rightpos;
    Servo leftpos;

    double  liftpower = 0;
    double mainbucketpower = 0;
    double intakePower = 0.48;
    double power0 = 0;
    double power1 = 0;
    double power2 = 0;
    double power3 = 0;
    double mainpower = 0;
    double rotatepower0 = 0;
    double rotatepower1 = 0;
    double forward;
    double clockwise;
    double right;
    double max;
    double k;



    boolean bCurrStateLbump;
    boolean bPrevStateLbump;
    boolean LBPon;
    boolean bCurrStateB;
    boolean bPrevStateB;
    boolean bIntakeOn;
    boolean bCurrStateC;
    boolean bPrevStateC;
    boolean CIntakeOn;
    boolean bCurrStateD;
    boolean bPrevStateD;
    boolean DIntakeOn;
    boolean bCurrStateX;
    boolean bPrevStateX;




   // will this work?
    @Nullable
    @Override
    public void runOpMode() {

        HardwareJoeBot2018 robot = new HardwareJoeBot2018();

        robot.init(hardwareMap, this);

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        rightpos = hardwareMap.get(Servo.class, "rightpos");
        leftpos = hardwareMap.get(Servo.class, "leftpos");




        // Wait for the start button
        telemetry.addData(">", "Ready to Go. Waiting for start" );
        telemetry.update();


        waitForStart();

        while(opModeIsActive()) {

            //forward = -gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            //right = -gamepad1.left_trigger + gamepad1.right_trigger;
            //clockwise = gamepad1.right_stick_x;

            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs


            forward = gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;




            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = -clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


            // Calculate motor power
            power0 = forward + clockwise + right;
            power1 = forward - clockwise - right;
            power2 = forward + clockwise - right;
            power3 = forward - clockwise + right;

            // Normalize Wheel speeds so that no speed exceeds 1.0
            max = Math.abs(power0);

            if (Math.abs(power1) > max) {
                max = Math.abs(power1);
            }
            if (Math.abs(power2) > max) {
                max = Math.abs(power2);
            }
            if (Math.abs(power3) > max) {
                max = Math.abs(power3);
            }

            if (max > 1) {
                power0 /= max;
                power1 /= max;
                power2 /= max;
                power3 /= max;
            }

            robot.motor0.setPower(power0);
            robot.motor1.setPower(power1);
            robot.motor2.setPower(power2);
            robot.motor3.setPower(power3);

            //------------------------------------------
            //-------------------------------------------

            /*
            rotatepower0 = -gamepad1.right_stick_x;
            rotatepower1 = gamepad1.right_stick_x;
            robot.motor0.setPower(rotatepower1);
            robot.motor0.setPower(rotatepower0);
            robot.motor1.setPower(rotatepower1);
            robot.motor2.setPower(rotatepower0);
                */
            //test out code above sometime





            // Map "power" variable to gamepad input
            mainbucketpower = gamepad2.left_stick_y * 0.75;
            robot.mainBucketMotor.setPower(mainbucketpower);

            liftpower = gamepad2.right_stick_y * 0.75;
            robot.liftMotor.setPower(liftpower);

            telemetry.addLine("operator i suppose");
            telemetry.update();



//-----------------------------------------------//


            // check the status of the left bumper button on  gamepad2.
            bCurrStateLbump = gamepad2.left_bumper;

            // check for button state transitions.
            if ((bCurrStateLbump == true) && (bCurrStateLbump != bPrevStateLbump)) {
                LBPon = !LBPon;
            }
            bPrevStateLbump = bCurrStateLbump;

            if (LBPon == true) {
                robot.liftbucketservo.setPosition(0);
            } else {
                robot.liftbucketservo.setPosition(.48);
            }

            telemetry.addLine("lift motor");
            telemetry.update();



//--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateB = gamepad2.a;

            // check for button state transitions.
            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                bIntakeOn = !bIntakeOn;

            }
            bPrevStateB = bCurrStateB;

            if (bIntakeOn == true) {
                robot.intakeMotor.setPower(intakePower);
            } else {
                robot.intakeMotor.setPower(0);

            }
            telemetry.addLine("intake motor");
            telemetry.update();


//--------------------------------------------------------------------------------------//
            // Toggle Intake Direction

            bCurrStateX = gamepad2.x;
            if ((bCurrStateX == true) && (bCurrStateX!=bPrevStateX)) {
                intakePower = intakePower * -1;
            }
            bPrevStateX = bCurrStateX;
            //--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateC = gamepad2.y;

            // check for button state transitions.
            if ((bCurrStateC == true) && (bCurrStateC != bPrevStateC)) {

                CIntakeOn = !CIntakeOn;

            }
            bPrevStateC = bCurrStateC;

            if (CIntakeOn == true) {
                rightpos.setPosition(0.7);
                leftpos.setPosition(0.2);
            } else {
                rightpos.setPosition(1);
                leftpos.setPosition(0);
            }
                telemetry.addLine("right and left pos");
            telemetry.update();

//--------------------------------------------------------------------------------------//

            // Display the current value

            telemetry.addData("Lift Bucket Motor Power", "%5.2f", liftpower);
            telemetry.addData("Main Bucket Motor Power", "%5.2f", mainpower);
            telemetry.addData("position", "%5.2f", robot.liftbucketservo.getPosition());
            telemetry.addData("right_position", "%5.2f", rightpos.getPosition());
            telemetry.addData("left_position", "%5.2f", leftpos.getPosition());


            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            idle();
        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}

//liftMotor
