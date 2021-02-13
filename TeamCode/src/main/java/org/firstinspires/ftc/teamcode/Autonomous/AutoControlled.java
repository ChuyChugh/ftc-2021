package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.Dependencies.UGRectDetector;

import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.AutoCommands;

@Autonomous(name="Autonomous")
public class AutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooterLeft, shooterRight, intake;
    private SimpleServo flicker, hopper;
    private SimpleServo grabber, gearboxL, gearboxR;
    public VoltageSensor voltageSensor;
    public UGRectDetector vision;
    private RevIMU imu;
    private PIDController pid = new PIDController(0.114, 0.001, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = new Motor(hardwareMap, "fR");
        frontLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        backLeft = new Motor(hardwareMap, "bL");
        shooterLeft = new Motor(hardwareMap, "shooterF");
        shooterRight = new Motor(hardwareMap, "shooterB");
        intake = new Motor(hardwareMap, "intake");
        hopper = new SimpleServo(hardwareMap, "hopper", -90, 180);
        grabber = new SimpleServo(hardwareMap, "grabber", -90, 180);
        hopper.setPosition(1-(35.0/270));
        flicker = new SimpleServo(hardwareMap, "flicker", -90, 180);

        gearboxL = new SimpleServo(hardwareMap, "gearboxL", -90, 180);
        gearboxR = new SimpleServo(hardwareMap, "gearboxR",-90,180);

        imu = new RevIMU(hardwareMap, "imu");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        vision = new UGRectDetector(hardwareMap, "camera");
        vision.init();
        vision.setTopRectangle(0.5486111111111112, 0.29140625);
        vision.setBottomRectangle(0.6458333333333334, 0.303125);
        vision.setRectangleSize(220, 40);

        gearboxR.setPosition(0);
        grabber.setPosition(0.5);
        gearboxL.setPosition(0);

        AutoCommands robot = new AutoCommands(frontLeft, frontRight, backLeft, backRight, new MotorGroup(shooterLeft, shooterRight), intake,
                grabber, flicker, voltageSensor, imu);
        robot.initialize();

        waitForStart();

        long startTime = System.currentTimeMillis();
        int visionDec = 0;
        while((System.currentTimeMillis() - startTime) < 3500) {
            telemetry.addData("Vision", vision.getStack());
            telemetry.update();
            visionDec = vision.getStack();
        }

        if (visionDec == 4){
            //telemetry for vision
            telemetry.addData("Vision", "4 Rings Detected");
            telemetry.update();

            //strafe 1 feet right
            robot.setTargetFeet(1, -1, -1, 1);
            robot.navigate(0.1);

            //move forward 10 feet
            robot.setTargetFeet(10, 10, 10, 10);
            robot.navigate(0.6);

            //strafe 1 feet left
            robot.setTargetFeet(-1, 1, 1, -1);
            robot.navigate(0.1);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //move backward 5 feet
            robot.setTargetFeet(-5, -5, -5, -5);
            robot.navigate(0.3);
        }

        if (visionDec == 0){
            //telemetry for vision
            telemetry.addData("Vision", "0 Rings Detected");
            telemetry.update();

            //move forward 6 feet
            robot.setTargetFeet(6, 6, 6, 6);
            robot.navigate(0.6);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //move backward 1 feet
            robot.setTargetFeet(-1, -1, -1, -1);
            robot.navigate(0.3);
        }

        if (visionDec == 1){
            //telemetry for vision
            telemetry.addData("Vision", "1 Ring Detected");
            telemetry.update();

            //strafe 1 feet right
            robot.setTargetFeet(1, -1, -1, 1);
            robot.navigate(0.1);

            //move forward 8 feet
            robot.setTargetFeet(8, 8, 8, 8);
            robot.navigate(0.6);

            //strafe 1 feet left
            robot.setTargetFeet(-1, 1, 1, -1);
            robot.navigate(0.1);

            //strafe left 3 feet
            robot.setTargetFeet(-3, 3, 3, -3);
            robot.navigate(0.1);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //move backward 2.5 feet
            robot.setTargetFeet(-2.5, -2.5, -2.5, -2.5);
            robot.navigate(0.3);

            //strafe right 2 feet
            robot.setTargetFeet(3, -3, -3, 3);
            robot.navigate(0.1);

            //move backward 1 feet
            robot.setTargetFeet(-1, -1, -1, -1);
            robot.navigate(0.3);
        }

        //strafe left 1 feet
        robot.setTargetFeet(-1.5, 1.5, 1.5,-1.5);
        robot.navigate(0.1);

        //get shooter up to speed
        robot.prepShooter();

        //rotate 180 degrees
        robot.setTargetRotation(180);
        robot.navigate(0.4);

        //shoot rings into top goal
        robot.shoot(8.5);

        //park on the line - REMOVE THIS PART IF WE DECIDE TO DO SECOND WOBBLE GOAL
        robot.setTargetFeet(-1, -1, -1, -1);
        robot.navigate(0.6);

        //stop motors
        robot.stopMotors();
        robot.finish();
    }
}
