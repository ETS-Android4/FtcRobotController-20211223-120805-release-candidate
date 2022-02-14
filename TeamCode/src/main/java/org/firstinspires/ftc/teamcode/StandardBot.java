/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a StandardBot.
 */
public class StandardBot {
    static final double COUNTS_PER_MOTOR_REV = 537.7; // 5203-2402-0019 goBilda Motor Encoder
    //static final double PPR_ARM_MOTOR = 3895.9; // 5202 Series Yellow Jacket Planetary Gear Motor (139:1 Ratio, 43 RPM, 3.3 - 5V Encoder)

    static final double WHEEL_DIAMETER_INCHES = 4.00; // 5202-0002-0139 goBilda Motor Encoder...Diameter of wheel in inches

    static final double DRIVE_TRAIN_GEAR_RATIO = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_TRAIN_GEAR_RATIO) /
            WHEEL_CIRCUMFERENCE;

    static final double WORM_GEAR_RATIO = 28.0;
    static final double ARM_WORM_COUNTS_PER_REV = COUNTS_PER_MOTOR_REV * WORM_GEAR_RATIO;

    static final double TURRET_LEFT_POSITION = 0.17;
    static final double TURRET_MIDDLE_POSITION = 0.49;
    static final double TURRET_RIGHT_POSITION = 0.83;
    static final double TURRET_INCREMENT = 0.01;

    // ARM LEVEL 4 is for capping the Team Shipping Element
    static final int ARM_LEVEL4 = (int) Math.round(0.40 * ARM_WORM_COUNTS_PER_REV);

    static final int ARM_LEVEL3 = (int)Math.round(0.23 * ARM_WORM_COUNTS_PER_REV);

    // level 3 = 796
    static final int ARM_LEVEL2 = (int)Math.round(0.160 * ARM_WORM_COUNTS_PER_REV);
    // level 2 = 579
    static final int ARM_LEVEL1 = (int)Math.round(0.090 * ARM_WORM_COUNTS_PER_REV);
    // level 1 = 288
    static final int ARM_LEVEL_REST = 0;
    static final int ARM_INCREMENT = (int) Math.round(0.005 * ARM_WORM_COUNTS_PER_REV);
    static final int ARM_POSITION_TOLERANCE = 5; // tolerate errors to within n ticks

    static final double EXTENDER_MIN_POSITION = 0.00;
    static final double EXTENDER_MAX_POSITION = 1.00;
    static final double EXTENDER_INCREMENT = 0.01;

    static final double MAGNET_START_POSITION = 0.00;
    static final double MAGNET_END_POSITION = 1.00;

    static final double OPTIMAL_DRIVE_SPEED = 0.7;
    static final double OPTIMAL_TURN_SPEED = 0.3;
    static final double OPTIMAL_STRAFE_SPEED = 0.45;

    static final double OPTIMAL_ARM_POWER = 1.0;
    static final double OPTIMAL_INTAKE_POWER = 0.7;
    static final double OPTIMAL_REST_POWER = 0.0;
    static final double OPTIMAL_CAROUSEL_POWER = 0.40;



    static final double TILE_SIZE = 24.0;
    private final ElapsedTime period = new ElapsedTime();

    public DcMotorImplEx stdRightFront = null; // max velocity = 2840 ticks
    public static final int RIGHT_FRONT_MAX_VELOCITY = 2840;

    public DcMotorImplEx stdRightRear = null;  // max velocity = 2840 ticks
    public static final int RIGHT_REAR_MAX_VELOCITY = 2840;

    public DcMotorImplEx stdLeftFront = null; // max velocity = 2880 ticks
    public static final int LEFT_FRONT_MAX_VELOCITY = 2880;

    public DcMotorImplEx stdLeftRear = null;  // max velocity = 2840 ticks
    public static final int LEFT_REAR_MAX_VELOCITY = 2840;

    static final int MAX_DRIVE_TRAIN_VELOCITY = 2840; // min value of all 4 wheels above for drive speed consistency

    public DcMotorImplEx stdCarouselMotor = null;  // max velocity = 2920 ticks
    public static final int CAROUSEL_MAX_VELOCITY = 2920;

    public DcMotorImplEx stdArmMotor = null; // max velocity = 3020 ticks
    public static final int ARM_MAX_VELOCITY = 3020;

    public Servo stdTurretServo = null;
    public Servo stdExtenderServo = null;
    public CRServo stdMagneticServo = null;
    public CRServo stdIntakeServo = null;
    public DistanceSensor stdDistanceSensor = null;
    public Rev2mDistanceSensor stdRevDistanceSensor = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public StandardBot() {
    }

    public void setDefaultMotorDirections() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }

        stdLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        stdLeftRear.setDirection(DcMotorEx.Direction.REVERSE);
        stdRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        stdRightRear.setDirection(DcMotorEx.Direction.FORWARD);

        stdArmMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotorsToRunWithoutEncoder() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }

        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdCarouselMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotorEncoders() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }

        stdLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //stdCarouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stdArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setMotorsToRunUsingEncoder() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }
        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorsToBrakeOnZeroPower() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }
        stdLeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdRightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        stdCarouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setAllMotorsToZeroPower() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }
        // Set all motors to zero power
        stdLeftFront.setPower(0);
        stdLeftRear.setPower(0);
        stdRightFront.setPower(0);
        stdRightRear.setPower(0);
        stdCarouselMotor.setPower(0);
        stdArmMotor.setPower(0);
    }

    public void setModeAllRTP() {
        if (hwMap == null) { // escape this function if the robot has not mapped to any motors
            return;
        }
        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setCustomPIDFCoefficients(){
        // Instructions from FIRST GLOBAL Motor PIDF Tuning Guide (Updated 3 September 2019)

        // Set Custom Velocity PIDF Coefficients

        // stdRightFront's max velocity = 2840 ticks
        // stdRightRear's max velocity = 2840 ticks
        // stdLeftFront's max velocity = 2880 ticks
        // stdLeftRear's max velocity = 2840 ticks

        // the min velocity of the 4 motors above is the max velocity of the drive train
        // driveTrain's max velocity = 2840 ticks

        // F (driveTrain) = 32767 / 2840 ticks
        double F = 11.5376760563;

        // P = 0.1 x F
        double P = 0.1 * F;

        // I = 0.1 x P
        double I = 0.1 * P;

        // D = 0
        double D = 0.0;

        stdRightFront.setVelocityPIDFCoefficients(P, I, D, F);
        stdRightRear.setVelocityPIDFCoefficients(P, I, D, F);
        stdLeftFront.setVelocityPIDFCoefficients(P, I, D, F);
        stdLeftRear.setVelocityPIDFCoefficients(P, I, D, F);

        // stdCarouselMotor's max velocity = 2920 ticks
        // F (carousel) = 32767 / 2920 ticks
        F = 11.2215753425;

        // P = 0.1 x F
        P = 0.1 * F;

        // I = 0.1 x P
        I = 0.1 * P;

        // D = 0
        D = 0.0;

        stdCarouselMotor.setVelocityPIDFCoefficients(P, I, D, F);

        // stdArmMotor's max velocity = 3020 ticks
        // F (arm) = 32767 / 3020 ticks
        F = 10.85;

        // P = 0.1 x F
        P = 0.1 * F;

        // I = 0.1 x P
        I = 0.1 * P;

        // D = 0
        D = 0.0;

        stdArmMotor.setVelocityPIDFCoefficients(P, I, D, F);

        // Set Custom Position PIDF Coefficients
        // Regardless of your maximum velocity, you can set the position PIDF values to:
        // see FIRST GLOBAL Motor PIDF Tuning Guide (Updated 3 September 2019)

        stdRightFront.setPositionPIDFCoefficients(5.0);
        stdRightRear.setPositionPIDFCoefficients(5.0);
        stdLeftFront.setPositionPIDFCoefficients(5.0);
        stdLeftRear.setPositionPIDFCoefficients(5.0);
        stdCarouselMotor.setPositionPIDFCoefficients(5.0);
        stdArmMotor.setPositionPIDFCoefficients(5.0);
    }

    public void resetArmToRestPosition() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        stdLeftFront = hwMap.get(DcMotorImplEx.class, "LeftFront");
        stdRightFront = hwMap.get(DcMotorImplEx.class, "RightFront");
        stdLeftRear = hwMap.get(DcMotorImplEx.class, "LeftRear");
        stdRightRear = hwMap.get(DcMotorImplEx.class, "RightRear");
        stdCarouselMotor = hwMap.get(DcMotorImplEx.class, "CarouselMotor");
        stdArmMotor = hwMap.get(DcMotorImplEx.class, "ArmMotor");

        // Define and initialize ALL installed servos.
        stdTurretServo = hwMap.get(Servo.class, "TurretServo");
        stdExtenderServo = hwMap.get(Servo.class, "ExtenderServo");
        stdMagneticServo = hwMap.get(CRServo.class, "MagneticServo");
        stdIntakeServo = hwMap.get(CRServo.class, "IntakeServo");

        stdDistanceSensor = hwMap.get(DistanceSensor.class, "DistanceSensor");
        stdRevDistanceSensor = (Rev2mDistanceSensor) stdDistanceSensor;

        resetMotorEncoders();
        setMotorsToRunWithoutEncoder();
        setDefaultMotorDirections();

        setMotorsToBrakeOnZeroPower();
        setAllMotorsToZeroPower();



    }
}
