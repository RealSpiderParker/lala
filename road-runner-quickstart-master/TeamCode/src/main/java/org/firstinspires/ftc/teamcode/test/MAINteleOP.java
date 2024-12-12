package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Config
@TeleOp(name = "MAINteleOP (Blocks to Java)")
public class MAINteleOP extends LinearOpMode {
    private PIDController controller;
    public static double p = .009, i = 0, d = .0001;
    public static double f = .1;
    public static int target = 0;
    private final double ticks_in_degrees = 780 / 180.0;
    private IMU imu_IMU;
    private DcMotor FrontL,BackL;
    private DcMotor FrontR;
    private DcMotor BackR;
    private DcMotor LeftM;
    private DcMotor RightM;
    private DcMotor Acuator;
    private Servo claw;
    private Servo pivot;
    private Servo wrist;
    private DcMotor ExtendMoto;

    double multiplier;
    YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
    double rotX;
    float x;
    float rx;
    double rotY;
    float y;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        LeftM = hardwareMap.get(DcMotor.class, "LeftM");
        RightM = hardwareMap.get(DcMotor.class, "RightM");
        Acuator = hardwareMap.get(DcMotor.class, "Acuator");
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        wrist = hardwareMap.get(Servo.class, "wrist");
        ExtendMoto = hardwareMap.get(DcMotor.class, "ExtendMoto");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // reberse
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        BackL.setDirection(DcMotor.Direction.REVERSE);
        RightM.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Acuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            imu_IMU.resetYaw();
            while (opModeIsActive()) {
                //public void loop
                controller.setPID(p, i, d);
                int armPos = LeftM.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
                double power = pid + ff;

                LeftM.setPower(power);
                RightM.setPower(power);

                // Put loop blocks here.
                _7ByawPitchRollAnglesVariable_7D = imu_IMU.getRobotYawPitchRollAngles();
                if (gamepad1.right_bumper) {
                    multiplier = 0.5;
                } else {
                    multiplier = 1;
                }
                gamepad();
                setPower();
                // Pressing Y will Reset the IMU for Field Centric
                if (gamepad1.y) {
                    imu_IMU.resetYaw();
                }
                // This is the code for the 3D printed Claw

                if (gamepad2.left_bumper) {
                    pivot.setPosition(1);
                } else if (gamepad2.right_bumper) {
                    pivot.setPosition(0.1);
                } else {
                    pivot.setPosition(0.3);
                }
                if (gamepad2.y) {
                    target = 420;
                }
                if (gamepad2.a) {
                    target = 15;
                }
                if (gamepad2.b) {
                    claw.setPosition(0.425);
                } else {
                    claw.setPosition(0);
                }
                if (gamepad2.x) {
                    wrist.setPosition((1));
                } else {
                    wrist.setPosition(0.375);
                }
                if (gamepad1.a) {
                    target = 700;
                }
                if (gamepad1.x) {
                    pivot.setPosition(1);
                }
                if (gamepad1.b) {
                    target 550;
                }
                LeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Acuator.setPower(-gamepad1.left_trigger);
                Acuator.setPower(gamepad1.right_trigger);
                Acuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ExtendMoto.setPower(gamepad2.right_stick_y);
                ExtendMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.update();
                data();
            }
        }
    }

    /**
     * Describe this function...
     */
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

    /**
     * Describe this function...
     */
    private void setPower() {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        FrontL.setPower((rotY + rotX + rx) * 1);
        FrontR.setPower(((rotY - rotX) - rx) * 1);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        BackL.setPower(((rotY - rotX) + rx) * 1);
        BackR.setPower(((rotY + rotX) - rx) * 1);
    }

    /**
     * Describe this function...
     */
    private void data() {
        telemetry.addData("IMU Yaw:", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES));
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Theta (Radians)", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.RADIANS));
        telemetry.addData("rx", rx);
        telemetry.addData("Multiplier (Speed)", multiplier);
        telemetry.update();
    }
}