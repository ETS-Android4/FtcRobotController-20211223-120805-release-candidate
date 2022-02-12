package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BLUE_AutoCarouselStorageUnit")

public class BLUE_AutoCarouselStorageUnit extends AutonomousBot {
    /* Declare OpMode members. */

    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //ACTUAL ACTIONS CODE

        rightStrafe(0.1);

        moveBackward(1.0, .3);

        spinCarousel(DcMotor.Direction.FORWARD, StandardBot.OPTIMAL_CAROUSEL_POWER, 5000);

        rightStrafe(1.2);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //robot.resetMotorEncoders();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d :%7d",
                robot.stdLeftFront.getCurrentPosition(),
                robot.stdLeftRear.getCurrentPosition(),
                robot.stdRightFront.getCurrentPosition(),
                robot.stdRightRear.getCurrentPosition(),
                robot.stdArmMotor.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
