package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoEncoderDriveTest")


public class AutoEncoderDriveTest extends AutonomousBot {
    AutoEncoderDriveTest(){ }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing HardwareMap");
        robot.init(hardwareMap);

        telemetry.addData("Status", "Set TurretServo to MIDDLE_POSITION");
        robot.stdTurretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);

        telemetry.addData("Status", "Set ExtenderServo to MAX_POSITION");
        robot.stdExtenderServo.setPosition(StandardBot.EXTENDER_MAX_POSITION);

        telemetry.update();

        sleep(5000);
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Moving forward 1 tile.");
            moveForward(1.0);

            telemetry.addData("Status", "Right strafe 1 tile.");
            rightStrafe(1.0);

            telemetry.addData("Status", "Moving backward 1 tile.");
            moveBackward(1.0);

            telemetry.addData("Status", "Left strafe 1 tile");
            leftStrafe(1.0);

            telemetry.update();
        }
    }
}
