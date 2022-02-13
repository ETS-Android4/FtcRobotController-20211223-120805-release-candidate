package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArmMotor", group = "Concept")

public class TestArmMotor extends LinearOpMode {
    double power;

    // Define class members
    DcMotor motor;

    @Override
    public void runOpMode() {

        // Connect to servo
        // Change the text in quotes to match any servo name on your robot.
        motor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Wait for the start button
        telemetry.addData(">", "Press Start to test motor.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            power = gamepad1.left_stick_y;
            motor.setPower(power);

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.update();
        }
    }
}
