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
    private ButtonReader incrementUp, incrementDown, pUp, pDown, iUp, iDown, dUp, dDown;

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

        pUp = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        pDown = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        iUp = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        iDown = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT);
        dUp = new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        dDown = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        incrementUp = new ButtonReader(gamepad, GamepadKeys.Button.A);
        incrementDown = new ButtonReader(gamepad, GamepadKeys.Button.B);

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
        boolean just_pressed = false;

        while(opModeIsActive() && !isStopRequested()){
            if (pUp.wasJustPressed()){
                p += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (pDown.wasJustPressed()){
                p -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (iUp.wasJustPressed()){
                i -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (iDown.wasJustPressed()){
                i += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (dUp.wasJustPressed()){
                d -= increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (dDown.wasJustPressed()){
                d += increment;
                shooterF.setVeloCoefficients(p, i , d);
                shooterB.setVeloCoefficients(p, i , d);
                just_pressed = true;
            }
            if (gamepad1.a){
                increment = 0.01;
                just_pressed = true;
            }
            if (gamepad1.b) {
                increment = 0.1;
                just_pressed = true;
            }

            shooterB.resetEncoder();
            shooterF.resetEncoder();

            long startTime = System.currentTimeMillis();
            long elapsedTime = 0L;

            while (just_pressed && elapsedTime < 1000 && opModeIsActive() && !isStopRequested()){
                elapsedTime = (new Date()).getTime() - startTime;
            }
            
            just_pressed = false;

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
