package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RED_AutoCarouselWarehouse")

public class RED_AutoCarouselWarehouse extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;          // 5203-2402-0019 goBilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;            // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77952756;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.1;
    static final double STRAFE_SPEED = 0.5;
    static final double TILE_SIZE = 24.0;            // in inches
    private final ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    StandardBot robot = new StandardBot();

    public void stopAllMotion() {
        robot.stdLeftFront.setPower(0);
        robot.stdLeftRear.setPower(0);
        robot.stdRightFront.setPower(0);
        robot.stdRightRear.setPower(0);
    }

    public void stopReset() {
        robot.stdLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setAllMotorPower(double power) {
        robot.stdLeftFront.setPower(power);
        robot.stdLeftRear.setPower(power);
        robot.stdRightFront.setPower(power);
        robot.stdRightRear.setPower(power);
    }

    public void setModeAllRTP() {
        robot.stdLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.stdLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.stdRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.stdRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void shutOffRTP() {
        robot.stdLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stdLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stdRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stdRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRight() {
        encoderDrive(TURN_SPEED, TILE_SIZE - 5, -(TILE_SIZE - 5), 5.0);
    }

    public void turnLeft() {
        encoderDrive(TURN_SPEED, -(TILE_SIZE - 5), TILE_SIZE - 5, 5.0);
    }

    public void moveForward(double nTiles) {
        encoderDrive(DRIVE_SPEED, -nTiles * TILE_SIZE, -nTiles * TILE_SIZE, 5.0);
    }

    public void moveBackward(double nTiles) {
        encoderDrive(DRIVE_SPEED, nTiles * TILE_SIZE, nTiles * TILE_SIZE, 5.0);
    }

    public void spinCarousel(DcMotorEx.Direction direction, double power, int time) {
        robot.stdCarouselMotor.setDirection(direction);
        robot.stdCarouselMotor.setPower(power);
        sleep(time);
        robot.stdCarouselMotor.setPower(0);
    }

    public void liftArm(int amount, double power) {
        int finalAmount = 0;

        if (opModeIsActive()) {
            finalAmount = robot.stdArmMotor.getCurrentPosition() + amount;
            robot.stdArmMotor.setTargetPosition(finalAmount);
            robot.stdArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.stdArmMotor.setPower(power);
            robot.stdArmMotor.setPower(0);
            robot.stdArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void leftStrafe(double nAmounts) {
        encoderLeftStrafe(STRAFE_SPEED, nAmounts * TILE_SIZE, nAmounts * TILE_SIZE, 5.0);
    }

    public void rightStrafe(double nAmounts) {
        encoderRightStrafe(STRAFE_SPEED, nAmounts * TILE_SIZE, nAmounts * TILE_SIZE, 5.0);
    }

    public void collectGameElement(double power, int time) {
        robot.stdIntakeServo.setPower(power);
        sleep(time);
        robot.stdIntakeServo.setPower(0);
    }

    public void regurgitateGameElement(double power, int time) {
        robot.stdIntakeServo.setPower(-power);
        sleep(time);
        robot.stdIntakeServo.setPower(0);
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.stdLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.stdArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.stdLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.stdLeftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.stdRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.stdRightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.stdArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d :%7d",
                robot.stdLeftFront.getCurrentPosition(),
                robot.stdLeftRear.getCurrentPosition(),
                robot.stdRightFront.getCurrentPosition(),
                robot.stdRightRear.getCurrentPosition(),
                robot.stdArmMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //codes here
        moveBackward(1.0);
        spinCarousel(DcMotorEx.Direction.REVERSE, 0.5, 5000);
        moveForward(7.0);
        collectGameElement(1.0, 3000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.stdLeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.stdRightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.stdLeftFront.setTargetPosition(newLeftTarget);
            robot.stdLeftRear.setTargetPosition(newLeftTarget);
            robot.stdRightFront.setTargetPosition(newRightTarget);
            robot.stdRightRear.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            setModeAllRTP();

            // reset the timeout time and start motion.
            runtime.reset();
            robot.stdLeftFront.setPower(Math.abs(speed));
            robot.stdLeftRear.setPower(Math.abs(speed));
            robot.stdRightFront.setPower(Math.abs(speed));
            robot.stdRightRear.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.stdLeftFront.isBusy() && robot.stdLeftRear.isBusy() && robot.stdRightFront.isBusy() && robot.stdRightRear.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d, %7d :%7d",
                        robot.stdLeftFront.getCurrentPosition(),
                        robot.stdLeftRear.getCurrentPosition(),
                        robot.stdRightFront.getCurrentPosition(),
                        robot.stdRightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotion();

            // Turn off RUN_TO_POSITION
            shutOffRTP();

            sleep(250);   // optional pause after each move
            stopReset();
        }
    }

    public void encoderLeftStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.stdLeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.stdRightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.stdLeftFront.setTargetPosition(newLeftTarget);
            robot.stdLeftRear.setTargetPosition(-newLeftTarget);
            robot.stdRightFront.setTargetPosition(-newRightTarget);
            robot.stdRightRear.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            setModeAllRTP();

            // reset the timeout time and start motion.
            runtime.reset();
            robot.stdLeftFront.setPower(Math.abs(speed));
            robot.stdLeftRear.setPower(Math.abs(speed));
            robot.stdRightFront.setPower(Math.abs(speed));
            robot.stdRightRear.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.stdLeftFront.isBusy() && robot.stdLeftRear.isBusy() && robot.stdRightFront.isBusy() && robot.stdRightRear.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d, %7d :%7d",
                        robot.stdLeftFront.getCurrentPosition(),
                        robot.stdLeftRear.getCurrentPosition(),
                        robot.stdRightFront.getCurrentPosition(),
                        robot.stdRightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotion();

            // Turn off RUN_TO_POSITION
            shutOffRTP();

            sleep(250);// optional pause after each move
            stopReset();
        }
    }

    public void encoderRightStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.stdLeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.stdRightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.stdLeftFront.setTargetPosition(-newLeftTarget);
            robot.stdLeftRear.setTargetPosition(newLeftTarget);
            robot.stdRightFront.setTargetPosition(newRightTarget);
            robot.stdRightRear.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            setModeAllRTP();

            // reset the timeout time and start motion.
            runtime.reset();
            robot.stdLeftFront.setPower(Math.abs(speed));
            robot.stdLeftRear.setPower(Math.abs(speed));
            robot.stdRightFront.setPower(Math.abs(speed));
            robot.stdRightRear.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.stdLeftFront.isBusy() && robot.stdLeftRear.isBusy() && robot.stdRightFront.isBusy() && robot.stdRightRear.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d, %7d :%7d",
                        robot.stdLeftFront.getCurrentPosition(),
                        robot.stdLeftRear.getCurrentPosition(),
                        robot.stdRightFront.getCurrentPosition(),
                        robot.stdRightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotion();

            // Turn off RUN_TO_POSITION
            shutOffRTP();

            sleep(250);   // optional pause after each move
            stopReset();
        }
    }
}