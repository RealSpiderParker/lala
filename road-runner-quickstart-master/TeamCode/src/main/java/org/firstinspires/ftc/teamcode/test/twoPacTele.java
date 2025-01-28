package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class twoPacTele extends LinearOpMode {
    private IMU imu;
    private DcMotor bLeft,bRight,fLeft,fRight,armL,armR,EM1;
    private Servo rCl,lCl,intakeCl,wrist,rPiv,lPiv,scoop;

    double multiplier;
    YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
    double rotX;
    float x;
    float rx;
    double rotY;
    float y;

    //PID stuff...
    private PIDController controller;
    public static double p = 0.009, i = 0, d = 0.0001;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "bRight");
        EM1 = hardwareMap.get(DcMotor.class, "EM1");
        armR = hardwareMap.get(DcMotor.class, "armR");
        armL = hardwareMap.get(DcMotor.class, "armL");
        rCl = hardwareMap.get(Servo.class, "rCl");
        lCl = hardwareMap.get(Servo.class, "lCl");
        intakeCl = hardwareMap.get(Servo.class, "intakeCl");
        rPiv = hardwareMap.get(Servo.class, "rPiv");
        lPiv = hardwareMap.get(Servo.class, "lPiv");
        wrist = hardwareMap.get(Servo.class, "wrist");
        scoop = hardwareMap.get(Servo.class, "scoop");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p,i,d);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        EM1.setDirection(DcMotorSimple.Direction.REVERSE);
        armL.setDirection(DcMotor.Direction.REVERSE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {

            imu.resetYaw();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                _7ByawPitchRollAnglesVariable_7D = imu.getRobotYawPitchRollAngles();

                //pid stuff...
                controller.setPID(p,i,d);
                int armPos = armL.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;
                armR.setPower(power);
                armL.setPower(power);

                if (gamepad1.right_bumper) {
                    multiplier = 0.5;
                } else {
                    multiplier = 1;
                }
                if (gamepad2.right_bumper) {
                    lCl.setPosition(.15);
                    rCl.setPosition(.15);
                } else {
                    lCl.setPosition(0);
                    rCl.setPosition(.3);
                }
                if (gamepad2.dpad_up) {
                    target = 0;
                }
                if (gamepad2.dpad_right) {
                    target = 925;
                }
                if (gamepad2.dpad_down) {
                    target = 975;
                }
                if (gamepad2.b) {
                    wrist.setPosition(.64);
                } else {
                    wrist.setPosition(.29);
                }
                if (gamepad2.left_bumper) {
                    intakeCl.setPosition(.4);
                } else {
                    intakeCl.setPosition(.2);
                }
                if (gamepad2.x) {
                    scoop.setPosition(.2);
                } else {
                    scoop.setPosition(1);
                }
                gamepad();
                setPower();
                // Pressing Right Bumper will Reset the IMU for Field Centric
                if (gamepad1.a) {
                    imu.resetYaw();
                }
                telemetry.addData("pos", armPos);
                telemetry.addData("target", target);
                telemetry.update();
                data();
            }
        }
    }
    private void gamepad() {
        double theta;

        // Use left stick to drive and right stick to turn
        // You may have to negate the sticks. When you
        // negate a stick, negate all other instances of the stick
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;
        theta = -_7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES);
        // Calculated Values
        rotX = x * Math.cos(theta / 180 * Math.PI) - y * Math.sin(theta / 180 * Math.PI);
        rotY = x * Math.sin(theta / 180 * Math.PI) + y * Math.cos(theta / 180 * Math.PI);
    }
    private void setPower() {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        fLeft.setPower((rotY + rotX + rx) * multiplier);
        fRight.setPower(((rotY - rotX) - rx) * multiplier);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        bLeft.setPower(((rotY - rotX) + rx) * multiplier);
        bRight.setPower(((rotY + rotX) - rx) * multiplier);
    }
    private void data() {
        telemetry.addData("IMU Yaw:", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES));
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Theta (Radians)", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.RADIANS));
        telemetry.addData("rx", rx);
        telemetry.addData("Multiplier (Speed)", multiplier);
        telemetry.addData("EM1", EM1.getCurrentPosition());

        telemetry.update();
    }
}
////this is a new code