package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "RED_AutoDetectTeamElementBarcode")

public class RED_AutoDetectTeamElementBarcode extends AutonomousBot {

    private final int DEBUG_SLEEP_TIME = 1000;
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
        robot.stdExtenderServo.setPosition(StandardBot.EXTENDER_MAX_POSITION);
        setDefaultMotorDirections();



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

            telemetry.addData("Moving", "rightStrafe 1.1 tiles");
            rightStrafe(1.5);

            try {
                if (teamElementBarcodeLevel == 1) {
                    telemetry.addData("ARM", "lifting to level 1");
                    liftArm(StandardBot.ARM_LEVEL1);
                } else if (teamElementBarcodeLevel == 2) {
                    telemetry.addData("ARM", "lifting to level 2");
                    liftArm(StandardBot.ARM_LEVEL2);
                } else {
                    telemetry.addData("ARM", "lifting to level 3");
                    liftArm(StandardBot.ARM_LEVEL3);
                }


                telemetry.addData("Arm", "at position %5d", robot.stdArmMotor.getCurrentPosition());

                telemetry.addData("Moving", "forward 0.6 tile");
                moveForward(0.6);

                telemetry.addData("Status", "regurgitating game element");
                regurgitateGameElement(0.3, 3000);

                telemetry.addData("Status", "return arm to rest position");
                returnArmPosition();

                telemetry.addData("Status", "leftStrafe 2.35 tiles");
                // strafe left toward the storage unit wall
                leftStrafe(2.35);

                telemetry.addData("Status", "moveBackward .75 tile");
                moveBackward(.75);

                telemetry.addData("Status", "spin carousel in reverse");
                spinCarousel(DcMotorEx.Direction.REVERSE, 2.00, StandardBot.OPTIMAL_CAROUSEL_SPEED, 3000);

                telemetry.addData("Status", "moveForward 1 tile");
                moveForward(1);
                leftStrafe(.2);

                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("Error Caught", e.toString());
                telemetry.update();
            }
        }

    }
}
