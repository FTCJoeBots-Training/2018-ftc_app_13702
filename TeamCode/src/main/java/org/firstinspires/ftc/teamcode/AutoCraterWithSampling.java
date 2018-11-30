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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 * This is a test Autonomous code to check the workings of the "moveInches" and "rotate" commands
 * in the 2018 HardwareJoeBots class.
 *
 */

@Autonomous(name="13702 Depot Middle Mineral IMPORTANT", group="Testing")

public class AutoCraterWithSampling extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2018      robot   = new HardwareJoeBot2018();
    int mineralPath = -1;
    int x;

    // @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        robot.minLanderPos();

        robot.minLanderPos();

        /*
        for(x=0; x<100; x++) {

            // set mineralpath = tensorflow data
            mineralPath = robot.tflocate();

            if (mineralPath>=0) {
                // We know we've found a mineral
                x = 100;
            }
            sleep(50);
        }
        */

        mineralPath = 1;

        switch(mineralPath) {
            case 0:
                // The gold mineral is on the left
                telemetry.addLine("<<LEFT>>");
                telemetry.update();
                break;

            case 1:
                // the gold mineral is in the center
                telemetry.addLine("<<CENTER>>");
                telemetry.update();
                break;

            case 2:
                // the gold mineral is on the right
                telemetry.addLine("<<RIGHT>>");
                telemetry.update();
                break;

            case -1:
                // The gold mineral can't be found
                telemetry.addLine("MINERAL SAMPLING ERROR!!!");
                telemetry.update();
                break;
        }



        waitForStart();

        //lower the robot (raise the lift)
        //robot.hangLanderPos();

        //get off hook
        // robot.moveRobot(0,-6,0);

        // lower landing arm

        // Sample Minerals... Do we see gold?
        // 0 = left; 1 = center; 2=right



        // Based on mineral location, drive to depot.

        if (mineralPath == 0) {
            // left driving paths
            robot.moveRobot(0,-3,0);

            robot.moveInches(44,1,15);

            robot.moveInches( -3, .5, 15);

            sleep(2000);

            robot.mainBucketMotor.setPower(-0.75);

            robot.rotate(-90,.5);

            robot.moveInches(40,.5,15);

            sleep(2000);

            robot.rotate(-45,0.5);

            robot.moveInches(75,1,15);

            robot.forwardToggle();

            robot.moveInches(-75, 0.5, 15);



            robot.forwardToggle();
        } else if (mineralPath == -1) {
            robot.moveInches(44,1,15);

            robot.moveInches( -3, .5, 15);

            sleep(2000);

            robot.mainBucketMotor.setPower(-0.75);

            robot.rotate(-90,.5);

            robot.moveInches(34,.5,15);

            sleep(2000);

            robot.rotate(-45,0.5);

            robot.moveInches(75,1,15);

            robot.forwardToggle();

            robot.moveInches(-75, 0.5, 15);



            robot.forwardToggle();
        } else if (mineralPath == 1) {
            robot.moveInches(44,1,15);

            robot.moveInches( -3, .5, 15);

            sleep(2000);

            robot.mainBucketMotor.setPower(-0.75);

            robot.rotate(-90,.5);

            robot.moveInches(34,.5,15);

            sleep(2000);

            robot.rotate(-45,0.5);

            robot.moveInches(75,1,15);

            robot.forwardToggle();

            robot.moveInches(-75, 0.5, 15);


            // center driving paths
        } else if (mineralPath == 2) {
            // right driving paths
            robot.moveRobot(0,3,0);

            robot.moveInches(44,1,15);

            robot.moveInches( -3, .5, 15);

            sleep(2000);

            robot.mainBucketMotor.setPower(-0.75);

            robot.rotate(-90,.5);

            robot.moveInches(28,.5,15);

            sleep(2000);

            robot.rotate(-45,0.5);

            robot.moveInches(75,1,15);

            robot.forwardToggle();

            robot.moveInches(-75, 0.5, 15);
        }


        // Head to Crater for parking

        //robot.rotate(45,.25);

       /*
        robot.moveInches(43,1,15);

        robot.forwardToggle();
        sleep(2000);
        robot.mainBucketMotor.setPower(-0.75);
        robot.rotate(-135,.5);
        robot.moveRobot(0,2,0);
        sleep(2000);
        robot.moveInches(75,1,15);
        */

        /*
        robot.moveRobot(0,3,0);
        robot.moveInches(35,1,15);
        robot.moveInches(-35,1,15);
        robot.moveRobot(0,-6,0);
        robot.moveInches(35,1,15);
        robot.moveInches(-35,1,15);
        robot.rotate(70,.25);
        robot.moveInches(-40,1,15);
        /*
        robot.mainBucketMotor.setPower(-0.75);

        robot.forwardToggle();
        robot.moveInches(-85,1,15);

        //////////////////////////////////////////

        */

    }

}
