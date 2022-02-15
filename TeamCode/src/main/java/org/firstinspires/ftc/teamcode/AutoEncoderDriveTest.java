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

    @Override
    public void runOpMode() {

        try {
            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            telemetry.addData("Status", "Initializing HardwareMap");
            robot.init(hardwareMap);

            telemetry.addData("Status", "Setting Default Motor Directions");
            setDefaultMotorDirections();

            robot.resetDriveTrainEncoder();

            telemetry.addData("Status", "Set TurretServo to MIDDLE_POSITION");
            robot.stdTurretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);

            telemetry.addData("Status", "Set ExtenderServo to MAX_POSITION");
            robot.stdExtenderServo.setPosition(StandardBot.EXTENDER_MAX_POSITION);

            telemetry.update();

            waitForStart();

            telemetry.addData("Status", "Moving backward 5 tiles.");
            moveBackward(1.0);

            telemetry.addData("Status", "Left strafe 5 tiles");
            leftStrafe(1.0);

            telemetry.addData("Status", "Moving forward 5 tile.");
            moveForward(1.0);

            telemetry.addData("Status", "Right strafe 5 tiles.");
            rightStrafe(1.0);

            telemetry.addData("DriveTrain", "leftFront (%7.2f), rightFront (%7.2f)", robot.stdLeftFront.getVelocity(), robot.stdRightFront.getVelocity());
            telemetry.addData("DriveTrain", "leftRear (%7.2f), rightRear (%7.2f)", robot.stdLeftRear.getVelocity(), robot.stdRightRear.getVelocity());

            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Exception", e.toString());
            telemetry.update();
        }
    }
}
