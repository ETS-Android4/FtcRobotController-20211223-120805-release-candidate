package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestArm")

public class TestArm extends AutonomousBot{

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized Hardware Map");

        robot.stdTurretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);
        telemetry.addData("Status", "TurretServo current position is %7.2f", robot.stdTurretServo.getPosition());

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "To lift Arm to LEVEL 1");
            liftArm(StandardBot.ARM_LEVEL1);
            telemetry.addData("Status", "Arm is set to LEVEL 1");


            telemetry.addData("Status", "Returning Arm to rest position");
            returnArmPosition();

            telemetry.addData("Status", "To lift Arm to LEVEL 2");
            liftArm(StandardBot.ARM_LEVEL2);
            telemetry.addData("Status", "Arm is set to LEVEL 2");

            telemetry.addData("Status", "Returning Arm to rest position");
            returnArmPosition();

            telemetry.addData("Status", "To lift Arm to LEVEL 3");
            liftArm(StandardBot.ARM_LEVEL3);
            telemetry.addData("Status", "Arm is set to LEVEL 3");

            telemetry.addData("Status", "Returning Arm to rest position");
            returnArmPosition();

            telemetry.addData("Status", "Moving forward 1 tile.");
            moveForward(1.0);

            telemetry.addData("Status", "Right strafe 1 tile.");
            rightStrafe(1.0);

            telemetry.addData("Status", "Moving backward 1 tile.");
            moveBackward(1.0);

            telemetry.addData("Status", "Left strafe 1 tile");
            leftStrafe(1.0);

            telemetry.update();

            while (robot.isDriveTrainBusy()) {
                telemetry.addData("Motors", "leftFront (%7.2f), rightFront (%7.2f)", robot.stdLeftFront.getVelocity(), robot.stdRightFront.getVelocity());
                telemetry.addData("Motors", "leftRear (%7.2f), rightRear (%7.2f)", robot.stdLeftRear.getVelocity(), robot.stdRightRear.getVelocity());
                telemetry.update();
            }
            telemetry.update();

            sleep(10000);
        }
    }

}
