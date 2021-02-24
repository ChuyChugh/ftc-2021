package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="FINAL TELEOP")
public class RealTeleOp extends LinearOpMode {

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
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    double p = 0;
    double i = 0;
    double d = 0;

    @Override

    public void runOpMode() throws InterruptedException {


        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(fL, fR, bL, bR);


        shooterF = new Motor(hardwareMap, "shooterF");
        shooterB = new Motor(hardwareMap, "shooterB");
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
        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setVeloCoefficients(0, 0, 0);
        shooterF.setVeloCoefficients(0, 0, 0);
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


        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
        fR.encoder.setDirection(Motor.Direction.REVERSE);
        telemetry.update();

        while(opModeIsActive() && !isStopRequested()){

            drive.driveRobotCentric(
                    -gamepad.getLeftX(),
                    -gamepad.getLeftY(),
                    -gamepad.getRightX(), true
            );

            if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
                shooterB.set(1);
                shooterF.set(1);
            } else {
                shooterB.set(0);
                shooterF.set(0);
            }
            if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                intake.set(1);
            }else if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                intake.set(-1);
            }else{
                intake.set(0);
            }

            if (flickerController.wasJustPressed()) {
                flicker.setPosition(1);
                sleep(100);
                flicker.setPosition(0.5);
            }

            if (hopperController.getState()) {
                hopper.setPosition(1-(40.0/270));
            } else {
                hopper.setPosition(1-(10.0/270));
            }

            if (gearboxController.getState()) {
                gearboxL.setPosition(0.85);
                gearboxR.setPosition(0.85);
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

