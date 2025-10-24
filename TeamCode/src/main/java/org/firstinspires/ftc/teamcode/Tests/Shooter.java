package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter test by mii")
public class Shooter extends LinearOpMode {
    public void runOpMode(){
        //Initialize the hardware variables
        DcMotor ShooterR = hardwareMap.get(DcMotor.class, "shooterR");
        DcMotor ShooterL = hardwareMap.get(DcMotor.class, "shooterL");

        //Inverting one of the sides
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.FORWARD);

        //Declarar variable de poder del shooter
        double powerS;

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {


            //Dar poder a los motores
            if (gamepad1.right_bumper) {
                powerS = 0.8;
            } else {
                powerS = 0;
            }

            ShooterR.setPower(powerS);
            ShooterL.setPower(powerS);

            telemetry.addData("Velocidad Shooter: ", powerS);
            telemetry.update();
        }
    }
}
