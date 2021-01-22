package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.TeleBot;

@TeleOp(name = "Tune PID Constants")
public class PIDTune extends LinearOpMode {
    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake, shooter, grabberLift;
    RevIMU imu;
    MecanumDrive m_drive;
    VoltageSensor voltageSensor;
    TeleBot robot;
    CRServo flicker;
    SimpleServo grabber;
    PIDController pid = new PIDController(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        intake = new Motor(hardwareMap, "intake");
        shooter = new Motor(hardwareMap, "shooter");
        flicker = new CRServo(hardwareMap, "flicker");
        grabberLift = new Motor(hardwareMap, "grabberLift");
        grabber = new SimpleServo(hardwareMap, "grabber");

        imu = new RevIMU(hardwareMap, "imu");

        robot = new TeleBot(frontLeft, frontRight, backLeft, backRight, intake, shooter, grabberLift, grabber, imu, flicker);
        robot.initialize();

        waitForStart();
        double p = 0;
        double i = 0;
        double d = 0;
        double increment = 0.1;

        while (opModeIsActive() && !isStopRequested()) {
            pid.setSetPoint(10);
            shooter.resetEncoder();
            shooter.set(pid.calculate(shooter.getCurrentPosition()));

            telemetry.addData("Ticks Per Loop Run", shooter.getCurrentPosition());
            telemetry.addData("Increment Level", increment);
            telemetry.addData("P Constant", p);
            telemetry.addData("I Constant", i);
            telemetry.addData("D Constant", d);

            telemetry.update();

            if (gamepad1.dpad_up){
                p += increment;
                pid.setP(p);
                sleep(500);
            }
            if (gamepad1.dpad_down){
                p -= increment;
                pid.setP(p);
                sleep(500);
            }
            if (gamepad1.dpad_left){
                i -= increment;
                pid.setI(i);
                sleep(500);
            }
            if (gamepad1.dpad_right){
                i += increment;
                pid.setI(i);
                sleep(500);
            }
            if (gamepad1.left_bumper){
                d -= increment;
                pid.setD(d);
                sleep(500);
            }
            if (gamepad1.right_bumper){
                d += increment;
                pid.setD(d);
                sleep(500);
            }
            if (gamepad1.a){
                increment = 0.01;
            }
            if (gamepad1.b){
                increment = 0.1;
            }
        }
    }
}
