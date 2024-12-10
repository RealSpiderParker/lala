package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Config
@TeleOp
public class PID2ndTest extends OpMode {
    //all the "private" stuff
    // double multiplier;
    //    YawPitchRollAngles_7ByawPitchRollAnglesVariable_7D;
    //    double rotX;
    //    float x;
    //    double rotY;
    //    float y;
    //private DcMotor FrontL,BackL, FrontR,BackR,Actuator,ExtendMoto;
    private PIDController controller;
private IMU imu_IMU;
    private DcMotorEx motor1,motor2;

    public static double p = .009, i = 0, d = .0001;
    public static double f = .1;
    public static int target = 0;
    private final double ticks_in_degrees = 780 / 180.0;


    @Override
    public void init() {
controller = new PIDController(p, i, d);
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
motor1 = hardwareMap.get(DcMotorEx.class, "LeftM");
motor2 = hardwareMap.get(DcMotorEx.class, "RightM");
motor2.setDirection(DcMotorSimple.Direction.REVERSE);
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
