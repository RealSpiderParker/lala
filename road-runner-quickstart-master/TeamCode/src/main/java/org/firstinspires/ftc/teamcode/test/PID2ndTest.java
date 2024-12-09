package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class PID2ndTest extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degrees = 780 / 180.0;
     private DcMotorEx motor1,motor2;

    @Override
    public void init() {
controller = new PIDController(p, i, d);
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

motor1 = hardwareMap.get(DcMotorEx.class, "LeftM");
motor2 = hardwareMap.get(DcMotorEx.class, "RightM");
    }

    @Override
    public void loop() {
controller.setPID(p, i, d);
int armPos = motor1.getCurrentPosition();
double pid = controller.calculate(armPos, target);
double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

double power = pid + ff;

motor1.setPower(power);
motor2.setPower(power);
telemetry.addData("pos ", armPos);
telemetry.addData("target ", target);
telemetry.update();
    }
}
