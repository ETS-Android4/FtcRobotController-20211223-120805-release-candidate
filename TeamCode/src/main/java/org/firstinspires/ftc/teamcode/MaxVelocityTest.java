package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    Servo extenderServo;
    double currentVelocity;
    double maxVelocity = 0.0;
    double minVelocity = 0.0;
/*
 // Define and Initialize Motors
        stdLeftFront = hwMap.get(DcMotorImplEx.class, "LeftFront");
        stdRightFront = hwMap.get(DcMotorImplEx.class, "RightFront");
        stdLeftRear = hwMap.get(DcMotorImplEx.class, "LeftRear");
        stdRightRear = hwMap.get(DcMotorImplEx.class, "RightRear");
        stdCarouselMotor = hwMap.get(DcMotorImplEx.class, "CarouselMotor");
        stdArmMotor = hwMap.get(DcMotorImplEx.class, "ArmMotor");


 */


    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        extenderServo = hardwareMap.get(Servo.class, "ExtenderServo");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        extenderServo.setPosition(1.0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.left_stick_y != 0.0) {
                motor.setPower(-gamepad2.left_stick_y);
            }
            else {
                motor.setPower(0);
            }


            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            if (currentVelocity < minVelocity) {
                minVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.addData("minumum velocity", minVelocity);
            telemetry.update();
        }
    }
}
