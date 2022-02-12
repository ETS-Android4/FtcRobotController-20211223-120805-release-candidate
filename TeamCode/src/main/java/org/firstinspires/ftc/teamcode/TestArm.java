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

        telemetry.addData("Status", "To lift Arm to LEVEL 1");
        liftArm(StandardBot.ARM_LEVEL1);
        telemetry.addData("Status", "Arm is set to LEVEL 1");

        telemetry.addData("Status", "Returning Arm to rest position");
        returnArmPosition();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "To lift Arm to LEVEL 2");
            liftArm(StandardBot.ARM_LEVEL2);
            telemetry.addData("Status", "Arm is set to LEVEL 2");

            telemetry.addData("Status", "Returning Arm to rest position");
            returnArmPosition();

//            liftArm(StandardBot.ARM_LEVEL2);
//            returnArmPosition();

//            liftArm(StandardBot.ARM_LEVEL3);
//            returnArmPosition();

            telemetry.update();

            sleep(10000);
        }
    }

}
