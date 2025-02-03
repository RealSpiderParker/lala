package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Config
public class testservos extends LinearOpMode {
    private Servo rCl,lCl,intakeCl,wrist,rPiv,lPiv,scoop;
public static double lpiv=0;
public static double rpiv = 0;
public static int test = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        rCl = hardwareMap.get(Servo.class, "rCl");
        lCl = hardwareMap.get(Servo.class, "lCl");
        intakeCl = hardwareMap.get(Servo.class, "intakeCl");
        rPiv = hardwareMap.get(Servo.class, "rPiv");
        lPiv = hardwareMap.get(Servo.class, "lPiv");
        wrist = hardwareMap.get(Servo.class, "wrist");
        scoop = hardwareMap.get(Servo.class, "scoop");
        rPiv.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {


            if (test == 0) {
                lPiv.setPosition(lpiv);

            } else if (test == 1) {
                rPiv.setPosition(rpiv);
            } else if (test == 3) {
                lPiv.setPosition(lpiv);
                rPiv.setPosition(lpiv);

            }
        }
    }
}
