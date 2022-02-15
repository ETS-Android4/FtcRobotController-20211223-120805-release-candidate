package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArmMotor", group = "Concept")

public class TestArmMotor extends LinearOpMode {
    double velocity;

    // Define class members
    DcMotorEx motor;

    @Override
    public void runOpMode() {

        // Connect to servo
        // Change the text in quotes to match any servo name on your robot.
        motor = hardwareMap.get(DcMotorEx.class, "ArmMotor");

        // Wait for the start button
        telemetry.addData(">", "Press Start to test motor.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            velocity = gamepad1.left_stick_y * StandardBot.ARM_MAX_VELOCITY;

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(velocity);

            // Display the current value
            telemetry.addData("Motor velocity", "%5.2f", velocity);
            telemetry.update();
        }
    }
}
