package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "RED_AutoDriveStorageUnitToWarehouse")

public class RED_AutoDriveStorageUnitToWarehouse extends AutonomousBot {

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
        rightStrafe(0.3);
        moveForward(5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
