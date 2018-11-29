
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

//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="13702 lower ", group="Pushbot")
public class LiftandLowerMyBoi extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2018      robot   = new HardwareJoeBot2018();   // Use a Pushbot's hardware



  //  ColorSensor sensorColor;
   // DistanceSensor sensorDistance;


    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap, this);


       // telemetry.addData("Distance (INCH)",
         //       String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.INCH)));
        //telemetry.addData("Alpha", sensorColor.alpha());
        //telemetry.addData("Red  ", sensorColor.red());
        //telemetry.addData("Green", sensorColor.green());
        //telemetry.addData("Blue ", sensorColor.blue());


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(-459);
        robot.liftMotor.setPower(0.7);

        // Red Crater Code

        //PASTE YOUR OPMODE CODE HERE
//lander attach/ detach
//robot.raiseLift();
//robot.moveRobot(0,-5,0);
//robot.lowerLift();
// ready to start

//come down

        /*
        robot.hangLanderPos();
        robot.moveRobot(0,-3,0);
        robot.moveInches(1,0.5,10);
        robot.moveRobot(0, 3, 0);
        robot.minLanderPos();
        */








        //runtime.reset();
       //runtime.startTime();
       //while (runtime.seconds() < 2 && hsvValues[0] >= 20 && hsvValues[0] <= 40) {
         //  robot.moveRobot(0,-0.2,0);
          // }`1
           //robot.stop();

     /*  if (sensorDistance.getDistance(DistanceUnit.INCH) <= 4.00) {
           stop();
           encoderDrive(12, 12, 12, 5);
       } else {
           forward = 0;
           //right = gamepad1.left_stick_x;
           right = 0.2;
           clockwise = 0;

           power1 = forward + clockwise + right;
           power2 = forward - clockwise - right;
           power3 = forward + clockwise - right;
           power4 = forward - clockwise + right;

           robot.motor1.setPower(power1);
           robot.motor2.setPower(power2);
           robot.motor3.setPower(power3);
           robot.motor4.setPower(power4);


       }


*/
//       robot.rotate(90,.2);
  //     robot.moveInches(24,.25,10);
    //   robot.rotate(-135,.2);
       //robot.moveInches(36,.25,10);





    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {



        }
    }
}
