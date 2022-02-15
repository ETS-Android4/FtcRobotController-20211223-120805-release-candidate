package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DuoTeleOpMain", group = "Linear Opmode")
public class DuoTeleOpMain extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final double armPower = 0;
    private final double carouselPower = 0;
    /* Declare OpMode members. */
    StandardBot robot = new StandardBot();   // Use a StandardBot's hardware
    private DcMotorImplEx leftFront = null;
    private DcMotorImplEx leftRear = null;
    private DcMotorImplEx rightFront = null;
    private DcMotorImplEx rightRear = null;
    private DcMotorImplEx carouselMotor = null;
    private DcMotorImplEx armMotor = null;
    private Servo turretServo = null;
    private Servo extenderServo = null;
    private CRServo magneticServo = null;
    private CRServo intakeServo = null;
    // Initialize other variables
    private double leftFrontSpeed = 0;
    private double leftRearSpeed = 0;
    private double rightFrontSpeed = 0;
    private double rightRearSpeed = 0;
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;
    private double contPower;

    private DistanceSensor distanceSensor = null;
    private Rev2mDistanceSensor revDistanceSensor = null;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. 
        leftFront = robot.stdLeftFront;
        leftRear = robot.stdLeftRear;
        rightFront = robot.stdRightFront;
        rightRear = robot.stdRightRear;
        carouselMotor = robot.stdCarouselMotor;
        armMotor = robot.stdArmMotor;

        turretServo = robot.stdTurretServo;
        extenderServo = robot.stdExtenderServo;
        magneticServo = robot.stdMagneticServo;

        intakeServo = robot.stdIntakeServo;
        distanceSensor = robot.stdDistanceSensor;
        revDistanceSensor = robot.stdRevDistanceSensor;

        turretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting ArmMotor Encoder");    //
        //armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPositionTolerance(StandardBot.ARM_POSITION_TOLERANCE);
        armMotor.setTargetPosition(StandardBot.ARM_LEVEL_REST);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(StandardBot.ARM_MAX_VELOCITY);
        //armMotor.setPower(StandardBot.OPTIMAL_REST_POWER);

        extenderServo.setPosition(StandardBot.EXTENDER_MAX_POSITION);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("ArmMotor Encoder", "Starting at %7d",
                armMotor.getCurrentPosition());

        telemetry.addData("deviceName", distanceSensor.getDeviceName());

        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if (currentDistance != DistanceSensor.distanceOutOfRange) {
            telemetry.addData("Sensor distance", "%.01f in", currentDistance);
        } else {
            telemetry.addData("Sensor distance", "OUT OF RANGE!");
        }

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (currentDistance != DistanceSensor.distanceOutOfRange) {
                telemetry.addData("Sensor distance", "%.01f in", currentDistance);
            } else {
                telemetry.addData("Sensor distance", "OUT OF RANGE!");
            }
            // Setup a variable for each drive wheel to save power level for telemetry
            // Mecanum mode
            drive = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            leftFrontSpeed = (StandardBot.MAX_DRIVE_TRAIN_VELOCITY) * (drive - strafe - rotate);
            leftRearSpeed = (StandardBot.MAX_DRIVE_TRAIN_VELOCITY) * (drive + strafe - rotate);
            rightFrontSpeed = (StandardBot.MAX_DRIVE_TRAIN_VELOCITY) * (drive + strafe + rotate);
            rightRearSpeed = (StandardBot.MAX_DRIVE_TRAIN_VELOCITY) * (drive - strafe + rotate);

            // Send calculated power to wheels
            leftFront.setVelocity(leftFrontSpeed);
            leftRear.setVelocity(leftRearSpeed);
            rightFront.setVelocity(rightFrontSpeed);
            rightRear.setVelocity(rightRearSpeed);

            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f)", leftFrontSpeed, rightFrontSpeed);
            telemetry.addData("Motors", "leftRear (%.2f), rightRear (%.2f)", leftRearSpeed, rightRearSpeed);


            // Controls the TURRET

            double turretServoPosition = 0.0;

            // Turn to LEFT
            if (gamepad2.dpad_left && armMotor.getTargetPosition() >= StandardBot.ARM_LEVEL2) {
                turretServoPosition = turretServo.getPosition() - StandardBot.TURRET_INCREMENT;

                if (turretServoPosition > StandardBot.TURRET_LEFT_POSITION) {
                    turretServo.setPosition(turretServoPosition);
                } else {
                    turretServo.setPosition(StandardBot.TURRET_LEFT_POSITION);
                }
            }
            // Turn to MIDDLE
            else if (gamepad2.dpad_down && armMotor.getTargetPosition() >= StandardBot.ARM_LEVEL2) {
                // Turning from LEFT
                if (turretServo.getPosition() < StandardBot.TURRET_MIDDLE_POSITION) {
                    turretServoPosition = turretServo.getPosition() + StandardBot.TURRET_INCREMENT;

                    if (turretServoPosition < StandardBot.TURRET_MIDDLE_POSITION) {
                        turretServo.setPosition(turretServoPosition);
                    } else {
                        turretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);
                    }

                }
                // Turning from RIGHT
                else if (turretServo.getPosition() > StandardBot.TURRET_MIDDLE_POSITION) {
                    turretServoPosition = turretServo.getPosition() - StandardBot.TURRET_INCREMENT;

                    if (turretServoPosition > StandardBot.TURRET_MIDDLE_POSITION) {
                        turretServo.setPosition(turretServoPosition);
                    } else {
                        turretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);
                    }
                }
            }
            // Turn to RIGHT
            else if (gamepad2.dpad_right && armMotor.getTargetPosition() >= StandardBot.ARM_LEVEL2) {
                turretServoPosition = turretServo.getPosition() + StandardBot.TURRET_INCREMENT;

                if (turretServoPosition < StandardBot.TURRET_RIGHT_POSITION) {
                    turretServo.setPosition(turretServoPosition);
                } else {
                    turretServo.setPosition(StandardBot.TURRET_RIGHT_POSITION);
                }
            }

            telemetry.addData("Motors", "Turret Position %5.2f", turretServo.getPosition());

            int newArmMotorTarget;

            // Controls the ARM
            //double armPower = gamepad2.left_stick_y;
            // Allows gamepad2.left_stick to move the arm up to ARM_LEVEL4 max and down freely 
            if (gamepad2.left_stick_y > 0) // going DOWNWARD
            {
                armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                armMotor.setVelocity(-gamepad2.left_stick_y * StandardBot.ARM_MAX_VELOCITY);
            }
            else if (gamepad2.left_stick_y < 0) // going UPWARD
            {
                armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                armMotor.setVelocity(-gamepad2.left_stick_y * StandardBot.ARM_MAX_VELOCITY);

            }
            else if (gamepad2.y)  // Raise ARM to LEVEL 3
            {
                if (armMotor.getCurrentPosition() < StandardBot.ARM_LEVEL3) {
                    newArmMotorTarget = armMotor.getTargetPosition() + StandardBot.ARM_LEVEL3;
                }
                else {
                    newArmMotorTarget = StandardBot.ARM_LEVEL3;
                }

                armMotor.setTargetPosition(newArmMotorTarget);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(StandardBot.ARM_MAX_VELOCITY);
            }
            else if (gamepad2.b) // Raise ARM to LEVEL 2
            {
                if (armMotor.getCurrentPosition() < StandardBot.ARM_LEVEL2) {
                    newArmMotorTarget = armMotor.getTargetPosition() + StandardBot.ARM_LEVEL2;
                }
                else {
                    newArmMotorTarget = StandardBot.ARM_LEVEL2;
                }

                armMotor.setTargetPosition(newArmMotorTarget);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(StandardBot.ARM_MAX_VELOCITY);
            }
            // Raise ARM to LEVEL 1
            else if (gamepad2.a) // && turretServo.getPosition() == robot.TURRET_MIDDLE_POSITION)             
            {
                if (armMotor.getCurrentPosition() < StandardBot.ARM_LEVEL1) {
                    newArmMotorTarget = armMotor.getTargetPosition() + StandardBot.ARM_LEVEL1;
                }
                else {
                    newArmMotorTarget = StandardBot.ARM_LEVEL1;
                }

                armMotor.setTargetPosition(newArmMotorTarget);
                armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armMotor.setVelocity(StandardBot.ARM_MAX_VELOCITY);
            }
            // resents the encoder
            else if (gamepad2.x)// && turretServo.getPosition() == robot.TURRET_MIDDLE_POSITION) 
            {
                armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            else {
                armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                armMotor.setVelocity(0);
            }


            telemetry.addData("Motors", "ArmMotor targetPosition is %7d", armMotor.getTargetPosition());
            telemetry.addData("Motors", "ArmMotor currentPosition is %7d", armMotor.getCurrentPosition());
            telemetry.addData("Motors", "ArmMotor CURRENT is %5.2f milli-amps", armMotor.getCurrent(CurrentUnit.MILLIAMPS));


            // Controls the CAROUSEL SPINNER
            if (gamepad1.right_trigger > 0)  // Spin COUNTER-CLOCKWISE
            {
                carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                carouselMotor.setDirection(DcMotorEx.Direction.REVERSE);
                carouselMotor.setPower(StandardBot.OPTIMAL_CAROUSEL_POWER);
            } else if (gamepad1.left_trigger > 0) // Spin CLOCKWISE
            {
                carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
                carouselMotor.setPower(StandardBot.OPTIMAL_CAROUSEL_POWER);
            } else
                carouselMotor.setPower(0.0);

            // Controls the INTAKE SPINNER

            if (gamepad1.right_bumper || gamepad2.right_bumper) // Spin inward
                intakeServo.setPower(-StandardBot.OPTIMAL_INTAKE_POWER);
            else if (gamepad1.left_bumper || gamepad2.left_bumper)  // Spin outward
                intakeServo.setPower(StandardBot.OPTIMAL_INTAKE_POWER);
            else
                intakeServo.setPower(0.0); // Stop spinner

            telemetry.addData("CRServos", "Intake Spiner Power %5.2f", intakeServo.getPower());
            telemetry.addData("Motors", "Carousel Motor Power %5.2f", carouselMotor.getPower());

            // Controls the ExtenderServo (with magnetic tip)

            // Right trigger retracts the ExtenderServo
            if (gamepad2.right_stick_y < 0) {
                double extenderServoPosition = extenderServo.getPosition() + StandardBot.EXTENDER_INCREMENT;

                if (extenderServoPosition < StandardBot.EXTENDER_MAX_POSITION) {
                    extenderServo.setPosition(extenderServoPosition);
                } else {
                    extenderServo.setPosition(StandardBot.EXTENDER_MAX_POSITION);
                }

            }
            // Left trigger extends the ExtenderServo
            else if (gamepad2.right_stick_y > 0) {
                double extenderServoPosition = extenderServo.getPosition() - StandardBot.EXTENDER_INCREMENT;

                if (extenderServoPosition > StandardBot.EXTENDER_MIN_POSITION) {
                    extenderServo.setPosition(extenderServoPosition);
                } else {
                    extenderServo.setPosition(StandardBot.EXTENDER_MIN_POSITION);
                }
            }
            // Display the current position of the ExtenderServo
            telemetry.addData("ExtenderServo Position", "%5.2f", extenderServo.getPosition());

            // Controls the MagneticServo
//            if (gamepad2.right_stick_x > 0)
//                && armMotor.getTargetPosition() >= robot.ARM_LEVEL3
//                && extenderServo.getPosition() <= robot.EXTENDER_MIN_POSITION)
            if (gamepad2.start)
            {
                magneticServo.setPower(StandardBot.OPTIMAL_INTAKE_POWER);
            }
//            else if (gamepad2.right_stick_x < 0)
//                && armMotor.getTargetPosition() >= robot.ARM_LEVEL3
//              && extenderServo.getPosition() <= robot.EXTENDER_MIN_POSITION)
            else if (gamepad2.back)
            {
                magneticServo.setPower(-StandardBot.OPTIMAL_INTAKE_POWER);
            } else {
                magneticServo.setPower(0.0);
            }

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}
