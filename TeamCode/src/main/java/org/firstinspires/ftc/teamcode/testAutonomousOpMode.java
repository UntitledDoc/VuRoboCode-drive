
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class testAutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin and motor direction
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift = hardwareMap.dcMotor.get("leftLift"); //left lift motor
        rightLift = hardwareMap.dcMotor.get("rightLift"); //right lift motor

        leftIntake = hardwareMap.dcMotor.get("rightLift"); //left intake motor
        rightIntake = hardwareMap.dcMotor.get("rightLift"); //right intake motor
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE); //reverse one motor for intake


        armServo = hardwareMap.servo.get("armServo"); //servo for the rotating single bar lift (reverse 4bar lift from VEX adaptive)
        clawServo = hardwareMap.servo.get("clawServo");
        capstoneServo = hardwareMap.servo.get("capstoneServo");//servo to drop marker into crater
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
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
//END OF CODE EDIT

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
