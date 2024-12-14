package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "newDRIVE")
public class MecanumDrivingSample extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    private PIDController controller;
    public static double p = .009, i = 0, d = .0001;
    public static double f = .1;
    public static int target = 0;
    private final double ticks_in_degrees = 780 / 180.0;
     DcMotor Actuator;
     Servo claw;
     Servo pivot;
     Servo wrist;
     DcMotor ExtendMoto;
     DcMotor LeftM,RightM;


    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "FrontL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "FrontR", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "BackL", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "BackR", Motor.GoBILDA.RPM_435)


        );

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
//////////////////////// public void loop
            controller.setPID(p, i, d);
            int armPos = LeftM.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            LeftM.setPower(power);
            RightM.setPower(power);

            if (gamepad2.left_bumper) {
                pivot.setPosition(.6);
            } else if (gamepad2.right_bumper) {
                pivot.setPosition(-.1);
            } else {
                pivot.setPosition(0.3);
            }
            if (gamepad2.x) {
                claw.setPosition(.4);
            } else {
                claw.setPosition(0);
            }
            if (gamepad2.b) {
                wrist.setPosition(1);
            } else {
                wrist.setPosition(0.375);
            }
            if (gamepad2.dpad_down) {
                target = 15;
            }
            if (gamepad2.dpad_up) {
                target = 700;
            }
            if (gamepad2.dpad_left) {
                target-=15;
            }
            if (gamepad2.dpad_right)    {
                target = 500;
            }
            LeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Actuator.setPower(-gamepad1.left_trigger);
            Actuator.setPower(gamepad1.right_trigger);
            Actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ExtendMoto.setPower(gamepad2.right_stick_y);
            ExtendMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("pos ", armPos);
            telemetry.addData("target ", target);
            telemetry.update();
            data();
            /////////////////////////////////
            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

                // Below is a model for how field centric will drive when given the inputs
                // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
                // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
                // regardless of the heading.
                //
                //                   heading
                //                     /
                //            (0,1,0) /
                //               |   /
                //               |  /
                //            ___|_/_____
                //          /           /
                //         /           / ---------- (1,0,0)
                //        /__________ /

                // optional fifth parameter for squared inputs
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        -driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

        }
    private void data() {
        telemetry.update();
    }
    }
