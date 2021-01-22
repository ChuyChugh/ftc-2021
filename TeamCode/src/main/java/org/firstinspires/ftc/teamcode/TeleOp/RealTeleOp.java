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
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    private ButtonReader flickerController, incSpeedController, decSpeedController, shootPositionController;
    private ToggleButtonReader gearboxController, hopperController, grabberController;
    private double speedMultiplier;

    @Override

    public void runOpMode() throws InterruptedException {

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(fL, fR, bL, bR);
        SampleMecanumDrive driveRR = new SampleMecanumDrive(hardwareMap);
        driveRR.setPoseEstimate(new Pose2d(-24, 9, Math.toRadians(90)));

        shooterF = new MotorEx(hardwareMap, "shooterF");
        shooterB = new MotorEx(hardwareMap, "shooterB");
        intake = new Motor(hardwareMap, "intake");

        flicker = new SimpleServo(hardwareMap, "flicker");
        hopper = new SimpleServo(hardwareMap, "hopper");
        grabber = new SimpleServo(hardwareMap, "grabber");
        gearboxL = new SimpleServo(hardwareMap, "gearboxL");
        gearboxR = new SimpleServo(hardwareMap, "gearboxR");

        incSpeedController = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        decSpeedController = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        shootPositionController = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT);
        flickerController = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        gearboxController = new ToggleButtonReader(gamepad, GamepadKeys.Button.A);
        grabberController = new ToggleButtonReader(gamepad, GamepadKeys.Button.B);
        hopperController = new ToggleButtonReader(gamepad, GamepadKeys.Button.Y);

        gamepad = new GamepadEx(gamepad1);
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

        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterF.setVeloCoefficients(0, 0, 0);
        shooterB.setVeloCoefficients(0, 0, 0);

        hopper.setPosition(1-(30.0/270));
        gearboxR.setPosition(0.5);
        gearboxL.setPosition(0.5);
        grabber.setPosition(0);

        waitForStart();

        speedMultiplier = 0.5;

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
        fR.encoder.setDirection(Motor.Direction.REVERSE);

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();

        while(opModeIsActive() && !isStopRequested()){

            drive.driveRobotCentric(
                    -gamepad.getLeftX() * speedMultiplier,
                    -gamepad.getLeftY() * speedMultiplier,
                    -gamepad.getRightX() * 0.75 * speedMultiplier
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
                hopper.setPosition(1-(30.0/270));
            } else {
                hopper.setPosition(1);
            }

            if (gearboxController.getState()) {
                gearboxL.setPosition(1.0);
                gearboxR.setPosition(1.0);
            } else {
                gearboxL.setPosition(0.5);
                gearboxR.setPosition(0.5);
            }

            if (grabberController.getState()) {
                grabber.setPosition(0.5);
            } else {
                grabber.setPosition(0);
            }

            if (incSpeedController.wasJustPressed() && speedMultiplier < 1) {
                speedMultiplier += 0.25;
                telemetry.addData("Speed Multiplier", speedMultiplier);
                telemetry.update();
            }
            if (decSpeedController.wasJustPressed() && speedMultiplier > 0) {
                speedMultiplier -= 0.25;
                telemetry.addData("Speed Multiplier", speedMultiplier);
                telemetry.update();
            }

            if (shootPositionController.wasJustPressed()){
                Trajectory path = driveRR.trajectoryBuilder(new Pose2d())
                        .lineToSplineHeading(new Pose2d(-36, 63, Math.toRadians(90)))
                        .build();
                driveRR.followTrajectory(path);
            }

            hopperController.readValue();
            gearboxController.readValue();
            grabberController.readValue();
        }
    }
}

