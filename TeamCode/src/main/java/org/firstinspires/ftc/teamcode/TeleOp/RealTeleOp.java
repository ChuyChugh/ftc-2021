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

@TeleOp(name="FINAL TELEOP")
public class RealTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR, intake;
    private MotorEx shooterF, shooterB;
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
        gamepad = new GamepadEx(gamepad1);
        incSpeedController = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        decSpeedController = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        flickerController = new ButtonReader(gamepad, GamepadKeys.Button.X);
        gearboxController = new ToggleButtonReader(gamepad, GamepadKeys.Button.A);
        grabberController = new ToggleButtonReader(gamepad, GamepadKeys.Button.B);
        hopperController = new ToggleButtonReader(gamepad, GamepadKeys.Button.Y);


        time = new ElapsedTime();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        flicker.setInverted(true);
        flicker.setPosition(0.5);

        shooterF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hopper.setPosition(1-(35.0/270));
        gearboxR.setPosition(0.4);
        gearboxL.setPosition(0.4);
        grabber.setPosition(0);

        waitForStart();


        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
        fR.encoder.setDirection(Motor.Direction.REVERSE);
        telemetry.update();

        while(opModeIsActive() && !isStopRequested()){

            drive.driveRobotCentric(
                    -gamepad.getLeftX() * 1.5,
                    -gamepad.getLeftY(),
                    -gamepad.getRightX() * 0.75, true
            );

            if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                shooterB.set(0.75);
                shooterF.set(0.75);
            } else {
                shooterB.set(0);
                shooterF.set(0);
            }

            intake.set(-gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            intake.set(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            if (flickerController.wasJustPressed()) {
                flicker.setPosition(1);
                sleep(350);
                flicker.setPosition(0.5);
            }

            if (hopperController.getState()) {
                hopper.setPosition(1-(35.0/270));
            } else {
                hopper.setPosition(1);
            }

            if (gearboxController.getState()) {
                gearboxL.setPosition(0.67);
                gearboxR.setPosition(0.67);
            } else {
                gearboxL.setPosition(0.1);
                gearboxR.setPosition(0.1);
            }

            if (grabberController.getState()) {
                grabber.setPosition(0.6);
            } else {
                grabber.setPosition(0);
            }

            telemetry.addData("Poopy left", fL.getCurrentPosition());
            telemetry.addData("Poopy right", fR.getCurrentPosition());
            telemetry.addData("Poopy center", bL.getCurrentPosition());
            telemetry.update();
            hopperController.readValue();
            gearboxController.readValue();
            grabberController.readValue();
            incSpeedController.readValue();
            flickerController.readValue();
        }
    }
}

