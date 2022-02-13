package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "RED_AutoDetectTeamElementRight")

public class RED_AutoDetectTeamElementRight extends AutonomousBot {

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initVuforia();
        initTfod();
        robot.stdTurretServo.setPosition(StandardBot.TURRET_MIDDLE_POSITION);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        int teamElementBarcodeLevel = 0;
        teamElementBarcodeLevel = getTargetLevelFromBarcode("RED");


        telemetry.addData("Team Element Barcode Level", "%7d", teamElementBarcodeLevel);

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            teamElementBarcodeLevel = getTargetLevelFromBarcode("RED");

            telemetry.addData("Moving", "leftStrafe 0.4 tiles");

            if (teamElementBarcodeLevel == 1) {
                liftArm(StandardBot.ARM_LEVEL1);
                telemetry.addData("ARM", "lifting to level 1");
            } else if (teamElementBarcodeLevel == 2) {
                liftArm(StandardBot.ARM_LEVEL2);
                telemetry.addData("ARM", "lifting to level 2");
            } else {
                liftArm(StandardBot.ARM_LEVEL3);
                telemetry.addData("ARM", "lifting to level 3");
            }

            telemetry.addData("Arm", "at position %5d", robot.stdArmMotor.getCurrentPosition());

            telemetry.addData("Moving", "forward 0.6 tile");

            moveForward(1.5);

            leftStrafe(0.4);
            moveForward(0.6, .5);

            telemetry.addData("Status", "regurgitating game element");
            regurgitateGameElement(0.3, 3000);

            telemetry.addData("Status", "Sleeping for 1000 milliseconds");
            sleep(1000);

            telemetry.addData("Status", "Sleeping for 500 milliseconds");
            sleep(500);



            //telemetry.addData("Status", "return arm to rest position");
            //returnArmPosition();

            turnRight();
            rightStrafe(0.4);

            moveForward(1.5);

            collectGameElement(0.4,2000);

            moveBackward(0.2);

            moveForward(1.5);
            leftStrafe(0.3);

        }

    }
}
