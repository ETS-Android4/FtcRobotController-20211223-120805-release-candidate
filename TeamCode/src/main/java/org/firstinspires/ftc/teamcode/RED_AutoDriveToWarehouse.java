package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RED_AutoDriveToWarehouse")

public class RED_AutoDriveToWarehouse extends AutonomousBot
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




        moveForward(2.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




}