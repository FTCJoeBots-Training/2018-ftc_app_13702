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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Sample code to test mapping of one motor to the gamepad.
 */
@TeleOp(name = "Concept: Ramp Motor Speed", group = "Concept")
//@Disabled
public class EthansCode extends LinearOpMode {


    // Define class members

    DcMotor  liftMotor;
    DcMotor  mainBucketMotor;
    DcMotor  intakeMotor;
    Servo liftbucket;
    Servo rightpos;
    Servo leftpos;

    double  liftpower = 0;
    double mainpower = 0;
    double intakepower = 0;


    boolean bCurrStateLbump;
    boolean bPrevStateLbump;
    boolean LBPon;
    boolean bCurrStateB;
    boolean bPrevStateB;
    boolean bIntakeOn;
    boolean bCurrStateC;
    boolean bPrevStateC;
    boolean CIntakeOn;

    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        liftMotor = hardwareMap.get(DcMotor.class, "liftmotor");
        mainBucketMotor = hardwareMap.get(DcMotor.class, "mainbucketmotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        liftbucket = hardwareMap.get(Servo.class, "liftbucket");
        rightpos = hardwareMap.get(Servo.class, "rightpos");
        leftpos = hardwareMap.get(Servo.class, "leftpos");




        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            // Map "power" variable to gamepad input
            mainpower = gamepad1.left_stick_y;
            mainBucketMotor.setPower(mainpower / 1.5);

            liftpower = gamepad1.right_stick_y;
            liftMotor.setPower(liftpower / 2);


//-----------------------------------------------//


            // check the status of the left bumper button on  gamepad2.
            bCurrStateLbump = gamepad2.left_bumper;

            // check for button state transitions.
            if ((bCurrStateLbump == true) && (bCurrStateLbump != bPrevStateLbump)) {
                LBPon = !LBPon;
            }
            bPrevStateLbump = bCurrStateLbump;

            if (LBPon == true) {
                liftbucket.setPosition(.18);
            } else {
                liftbucket.setPosition(.48);
            }



//--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateB = gamepad2.a;

            // check for button state transitions.
            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                bIntakeOn = !bIntakeOn;

            }
            bPrevStateB = bCurrStateB;

            if (bIntakeOn == true) {
                intakeMotor.setPower(.75);
            } else {
                intakeMotor.setPower(0);

            }

//--------------------------------------------------------------------------------------//
            //--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateC = gamepad2.y;

            // check for button state transitions.
            if ((bCurrStateC == true) && (bCurrStateC != bPrevStateC)) {

                CIntakeOn = !CIntakeOn;

            }
            bPrevStateC = bCurrStateC;

            if (CIntakeOn == true) {
              rightpos.setPosition(.7);
              leftpos.setPosition(0);
            } else {
                rightpos.setPosition(0);
                leftpos.setPosition(.7);
                }

//--------------------------------------------------------------------------------------//

            // Display the current value

            telemetry.addData("Lift Bucket Motor Power", "%5.2f", liftpower);
            telemetry.addData("Main Bucket Motor Power", "%5.2f", mainpower);
            telemetry.addData("position", "%5.2f", liftbucket.getPosition());
            telemetry.addData("right_position", "%5.2f", rightpos.getPosition());
            telemetry.addData("left_position", "%5.2f", leftpos.getPosition());

            // telemetry.addData("Lift Bucket Servo", "%5.2f", liftbucketpos);


            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            idle();
        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}