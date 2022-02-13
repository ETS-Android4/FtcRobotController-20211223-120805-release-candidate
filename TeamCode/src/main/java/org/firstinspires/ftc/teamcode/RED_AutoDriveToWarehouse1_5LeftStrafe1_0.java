package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RED_AutoDriveToWarehouse1_5LeftStrafe1_0")

public class RED_AutoDriveToWarehouse1_5LeftStrafe1_0 extends AutonomousBot
{

    @Override
    public void runOpMode()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //codes here




        moveForward(0.6,0.3);
        leftStrafe(1.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




}