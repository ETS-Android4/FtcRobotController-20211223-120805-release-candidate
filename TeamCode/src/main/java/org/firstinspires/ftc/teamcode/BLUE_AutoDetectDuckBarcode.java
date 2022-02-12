package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BLUE_AutoDetectDuckBarcode")

public class BLUE_AutoDetectDuckBarcode extends AutonomousBot {

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
         */
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

        int duckBarcodeLevel = 0;


        duckBarcodeLevel = getTargetLevelFromBarcode("BLUE");
        telemetry.addData("Duck Barcode Level", "%7d", duckBarcodeLevel);

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            duckBarcodeLevel = getTargetLevelFromBarcode("BLUE");

            leftStrafe(1.3);

            telemetry.addData("Moving", "leftStrafe 1.2 tiles");

            moveForward(0.7, .5);

            telemetry.addData("Moving", "forward 0.7 tile");

            if (duckBarcodeLevel == 1) {
                liftArm(StandardBot.ARM_LEVEL1);
                telemetry.addData("ARM", "lifting to level 1");
            } else if (duckBarcodeLevel == 2) {
                liftArm(StandardBot.ARM_LEVEL2);
                telemetry.addData("ARM", "lifting to level 2");
            } else {
                liftArm(StandardBot.ARM_LEVEL3);
                telemetry.addData("ARM", "lifting to level 3");
            }

            telemetry.addData("Arm", "at position %5d", robot.stdArmMotor.getCurrentPosition());

            regurgitateGameElement(0.2, 3000);

            //returnArmPosition();

            sleep(500);
            // strafe right toward the storage unit wall
            rightStrafe(2.35);

            // backup toward the carousel
            moveBackward(.39, 0.3);

            spinCarousel(DcMotorEx.Direction.FORWARD, StandardBot.OPTIMAL_CAROUSEL_POWER, 3000);

            moveForward(3.5);

            rightStrafe(.2);
        }

    }
}
