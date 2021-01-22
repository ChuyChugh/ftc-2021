package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Dependencies.AutoCommands;

@Autonomous(name="Advanced Autonomous")
public class AdvancedAutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, intake, arm;
    private SimpleServo grabber, flicker;
    private RevIMU imu;

    private PIDController armPID = new PIDController(0, 0, 0);
    private PIDController drivetrainPID = new PIDController(0, 0, 0);
    private PIDController shooterPID = new PIDController(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = new Motor(hardwareMap, "fL");
        frontLeft = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bL");
        backLeft = new Motor(hardwareMap, "bR");
        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");
        arm = new Motor(hardwareMap, "grabberLift");
        grabber = new SimpleServo(hardwareMap, "grabber");
        flicker = new SimpleServo(hardwareMap, "flicker");
        imu = new RevIMU(hardwareMap, "imu");

        AutoCommands robot = new AutoCommands(frontLeft, frontRight, backLeft, backRight, shooter, intake, arm, grabber, flicker, imu);
        robot.initializeAdvancedAuto();

        waitForStart();

        //UG Contour Ring Pipeline


    }
}
