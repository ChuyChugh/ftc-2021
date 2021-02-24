package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Date;

@TeleOp(name="tune pid shooter")
public class PIDTune extends LinearOpMode {

    private Motor fL, fR, bL, bR, intake;
    private Motor shooterF, shooterB;
    private SimpleServo hopper, flicker,grabber, gearboxL, gearboxR;
    private ElapsedTime time;
    private RevIMU imu;
    private MecanumDrive drive;
    private VoltageSensor voltageSensor;
    private GamepadEx gamepad;
    private ButtonReader flickerController, incSpeedController, decSpeedController;
    private ToggleButtonReader gearboxController, hopperController, grabberController;

    @Override

    public void runOpMode() throws InterruptedException {

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(fL, fR, bL, bR);


        shooterF = new MotorEx(hardwareMap, "shooterF");
        shooterB = new MotorEx(hardwareMap, "shooterB");
        intake = new Motor(hardwareMap, "intake");

        flicker = new SimpleServo(hardwareMap, "flicker", -90,90);
        hopper = new SimpleServo(hardwareMap, "hopper", -90, 180);
        grabber = new SimpleServo(hardwareMap, "grabber", -90, 180);
        gearboxL = new SimpleServo(hardwareMap, "gearboxL", -90, 180);
        gearboxR = new SimpleServo(hardwareMap, "gearboxR",-90,180);

        time = new ElapsedTime();
        flicker.setInverted(true);
        flicker.setPosition(0.5);

        shooterF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hopper.setPosition(1-(40.0/270));
        gearboxR.setPosition(0.4);
        gearboxL.setPosition(0.4);
        grabber.setPosition(0);

        waitForStart();

        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setVeloCoefficients(0, 0, 0);
        shooterF.setVeloCoefficients(0, 0, 0);

        shooterF.set(0.5);
        shooterB.set(0.5);

        double p = 0;
        double i = 0;
        double d = 0;
        double increment = 0.1;
        while(opModeIsActive() && !isStopRequested()){

            if (gamepad1.dpad_up){
                p += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.dpad_down){
                p -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.dpad_left){
                i -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.dpad_right){
                i += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.left_bumper){
                d -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.right_bumper){
                d += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                sleep(500);
            }
            if (gamepad1.a){
                increment = 0.01;
            }
            if (gamepad1.b) {
                increment = 0.1;
            }

            shooterB.resetEncoder();
            shooterF.resetEncoder();
            long startTime = System.currentTimeMillis();
            long elapsedTime = 0L;

            while (elapsedTime < 1000 && opModeIsActive() && !isStopRequested()){
                elapsedTime = (new Date()).getTime() - startTime;
            }
            telemetry.addData("Ticks Per Second Back", shooterB.getCurrentPosition());
            telemetry.addData("Ticks Per Second Front", shooterF.getCurrentPosition());
            telemetry.addData("Increment Level", increment);
            telemetry.addData("P Constant", p);
            telemetry.addData("I Constant", i);
            telemetry.addData("D Constant", d);
            telemetry.update();
       }
    }
}


