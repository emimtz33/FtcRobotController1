package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ServoTest by Emiliano")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Inicializacion del servo
        Servo servoOne;
        servoOne =  hardwareMap.get(Servo.class,"servo one");
        //servoOne.setposition
        //limitar al servo
        servoOne.scaleRange(0.2,0.8);

        waitForStart();

        while(opModeIsActive()){
            servoOne.setPosition(); //Posicion del servo

            
        //Controlar servo con control
        if(gamepad1.circle){
            servoOne.setPosition(1);
            } else if (gamepad1.triangle) {
                servoOne.setPosition(0);
            }


        //Telemetria de posicion del servo
            telemetry.addData("Servo Position: ", servoOne.getPosition());
            telemetry.update();
        }

    }
}
