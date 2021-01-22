package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.TeleBot;

@TeleOp(name = "Tune Shooter")
public class PIDTest extends LinearOpMode {
    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake, shooter, grabberLift;
    RevIMU imu;
    MecanumDrive m_drive;
    VoltageSensor voltageSensor;
    TeleBot robot;
    CRServo flicker;
    SimpleServo grabber;
    PIDController pid = new PIDController(0.114, 0.001, 0);

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

        m_drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = new TeleBot(frontLeft, frontRight, backLeft, backRight, intake, shooter, grabberLift, grabber, imu, flicker);
        robot.initialize();

        waitForStart();
        double point = 0;

        while (opModeIsActive() && !isStopRequested()) {
            pid.setSetPoint(point);
            shooter.resetEncoder();
            shooter.set(pid.calculate(shooter.getCurrentPosition()));

            if (gamepad1.dpad_up){
                point += 1;
                sleep(500);
            }
            if (gamepad1.dpad_down){
                point -= 1;
                sleep(500);
            }

            telemetry.addData("Speed Selected", point);
            telemetry.update();
        }
    }


}
