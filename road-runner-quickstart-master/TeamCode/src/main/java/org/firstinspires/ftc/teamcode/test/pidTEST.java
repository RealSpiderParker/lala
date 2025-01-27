package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class pidTEST extends OpMode {
    private PIDController controller;
    public static double p = 0.009, i = 0, d = 0.0001;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;
    private DcMotor armL,armR;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armL = hardwareMap.get(DcMotorEx.class, "armL");
        armR = hardwareMap.get(DcMotorEx.class, "armR");
        armL.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = armL.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        armR.setPower(power);
        armL.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}
