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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class testerCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive, leftLift, rightLift, leftIntake, rightIntake;


    private Servo armServo, capstoneServo, clawServo, sensorArm; //4 servos
    double initialCapstoneServoPosition = 0.0; //change according to whatever
    double initialArmServoPosition = 0.0; //change according to to whatever
    double initialClawServoPosition = 0.5; //gonna start halfway from its position, can't start full
    double initialSensorArmPosition = 0;

    private DistanceSensor sensorRangeLeftFront, sensorRangeLeftBack, sensorRangeRightFront, sensorRangeRightBack, sensorRangeArm;

    private float drivePowerRY, drivePowerRX, drivePowerLY, drivePowerLX;

    //encoder stuff for the lift
    static final double     COUNTS_PER_MOTOR_GOBILDA435    = 753.2 ;    // GoBilda 435 RPM counts
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBILDA435 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     DRIVE_SPEED             = 0.3; //not needed for my encoder code
    //static final double     TURN_SPEED              = 0.3; //not needed for my encoder code

    int stoneLevel = 0; //hopefully this doesn't get looped
    long setTime = System.nanoTime();
    boolean hasRun = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        //rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        //leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        //leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");
        //leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin and motor direction
        //leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift = hardwareMap.dcMotor.get("leftLift"); //left lift motor
        rightLift = hardwareMap.dcMotor.get("rightLift"); //right lift motor
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake = hardwareMap.dcMotor.get("rightLift"); //left intake motor
        rightIntake = hardwareMap.dcMotor.get("rightLift"); //right intake motor
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE); //reverse one motor for intake


        //armServo = hardwareMap.servo.get("armServo"); //servo for the rotating single bar lift (reverse 4bar lift from VEX adaptive)
        //clawServo = hardwareMap.servo.get("clawServo");
        //capstoneServo = hardwareMap.servo.get("capstoneServo");//servo to drop marker into crater
        //foundation will be moved by a 3d printed part on the lift going to base level

        sensorRangeLeftFront = hardwareMap.get(DistanceSensor.class, "sensorRangeLeftFront");
        sensorRangeLeftBack = hardwareMap.get(DistanceSensor.class, "sensorRangeLeftBack");
        sensorRangeRightFront = hardwareMap.get(DistanceSensor.class, "sensorRangeRightFront");
        sensorRangeRightBack = hardwareMap.get(DistanceSensor.class, "sensorRangeRightBack");

        //encoder hardware, reset position
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //finishes reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftLift.getCurrentPosition(),
                rightLift.getCurrentPosition());
        telemetry.update();


        //</editor-fold>
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        armServo.setPosition(initialArmServoPosition);
        capstoneServo.setPosition(initialCapstoneServoPosition);
        clawServo.setPosition(initialClawServoPosition);
        sensorArm.setPosition(initialSensorArmPosition);runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //stone code
        if (gamepad1.a)
        {
            stoneLevel += 1;
            telemetry.addData("Stone level: ", stoneLevel);
        }
        if (gamepad1.b);
        {
            stoneLevel -= 1;
            telemetry.addData("Stone level: ", stoneLevel);
        }

        //Raise lift code
        if (gamepad1.right_trigger > 0)
        {
            //PART 1
            clawServo.setPosition(0.55); //clamps the claw servo on the block slightly (change if wrong direction or whatever)

            int leftInches = stoneLevel*5 + 5; //level 0 is no stones in tower, but needs to clear robot
            int rightInches = stoneLevel*5 + 5;

            // Determine new target position, and pass to motor controller
            int newLeftTarget =  leftLift.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRightTarget =  rightLift.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftLift.setTargetPosition(newLeftTarget);
            rightLift.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftLift.setPower(Math.abs(1));
            rightLift.setPower(Math.abs(1));

            //PART 2, arm rotates after it has ascended enough
            if(System.nanoTime() - setTime > (2000000000) && !hasRun) //nano is 10^6 more precise than milli-time
            {
                //Will only run after 2 second, and will only run once
                hasRun = true; //changes conditional
                armServo.setPosition(1); //rotates arm after a 2 second delay
            }

            // PART 3, goes down for height of clearance
            leftInches = -4;
            rightInches = -4;

            // Determine new target position, and pass to motor controller
            newLeftTarget =  leftLift.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget =  rightLift.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftLift.setTargetPosition(newLeftTarget);
            rightLift.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftLift.setPower(Math.abs(1));
            rightLift.setPower(Math.abs(1));
            hasRun = false;

            if (stoneLevel > 0) //only works if there is already a tower to guide to
            {
                if (System.nanoTime() - setTime > (1000000000) && !hasRun) //nano is 10^6 more precise than milli-time
                {
                    //Will only run after 1 second, and will only run once
                    hasRun = true; //changes conditional
                    sensorArm.setPosition(1); //rotates Sensor Arm after a 1 second delay
                }
                double adjustmentDistance = 15 - sensorRangeLeftFront.getDistance(DistanceUnit.CM); //to gain certainty about our position if and only if there is a tower
                while (adjustmentDistance > 0.5)
                {
                    adjustmentDistance = 15 - sensorRangeLeftFront.getDistance(DistanceUnit.CM);
                    leftFrontDrive.setPower(-.25);
                    rightFrontDrive.setPower(-.25);
                    leftBackDrive.setPower(.25);
                    rightBackDrive.setPower(.25);
                }
                while (adjustmentDistance < -0.5)
                {
                    adjustmentDistance = 15 - sensorRangeLeftFront.getDistance(DistanceUnit.CM);
                    leftFrontDrive.setPower(.25);
                    rightFrontDrive.setPower(.25);
                    leftBackDrive.setPower(-.25);
                    rightBackDrive.setPower(-.25);
                }
                hasRun = false;
            }

            //i think this is complete, probably ends after it completes task, but idk
        }
        //lower the lift code
        /*
        if (gamepad1.right_bumper)
        {
            hasRun = false;

            int leftInches = -(stoneLevel*5 + 1); //level 0 is no stones in tower, so just clear 1 inch of foundation, each stone past is 5 inches
            int rightInches = -(stoneLevel*5 + 1);

            if(stoneLevel == 0) //if stone on the 0th level, then we need to go up before going in, if not ignore
            {
                leftInches = 4;
                rightInches = 4;

                int newLeftTarget =  leftLift.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                int newRightTarget =  rightLift.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                leftLift.setTargetPosition(newLeftTarget);
                rightLift.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftLift.setPower(Math.abs(1));
                rightLift.setPower(Math.abs(1));

                if(System.nanoTime() - setTime > (2000000000) && !hasRun) //nano is 10^6 more precise than milli-time
                {
                    //Will only run after 2 second, and will only run once
                    hasRun = true; //changes conditional
                    clawServo.setPosition(0.5); //opens claw after a 2 second delay
                }
            }
            armServo.setPosition(0); //rotate the arm back into the robot, then goes down

            // Determine new target position, and pass to motor controller
            int newLeftTarget =  leftLift.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRightTarget =  rightLift.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftLift.setTargetPosition(newLeftTarget);
            rightLift.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftLift.setPower(Math.abs(1));
            rightLift.setPower(Math.abs(1));
            hasRun = false; //false for another loop
        }



        //claw code
        //IF we had the correct motor OR odometry, I could combine the claw code and the lift code into a single command instead of two
        if (gamepad1.x && armServo.getPosition() >0.98)
        {
            clawServo.setPosition(0.53);
        }
*/



        //intake code, have to press dpad up and down, although it is a bit awkward for move+intake
        if (gamepad1.dpad_up)
        {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        if (gamepad1.dpad_down) //spit the block out if for some reason something is wrong
        {
            leftIntake.setPower(-0.5);
            rightIntake.setPower(-0.5);
        }

        //</editor-fold>

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
