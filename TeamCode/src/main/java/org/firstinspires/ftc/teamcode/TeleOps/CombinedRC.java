package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Combined Robot Code")
public class CombinedRC extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Estado", "Funcional");
        telemetry.update();

        //Initialize the hardware variables
        DcMotor Intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor ShooterR = hardwareMap.get(DcMotor.class, "shooterR");
        DcMotor ShooterL = hardwareMap.get(DcMotor.class, "shooterL");
        DcMotor FrenteIzquierdo = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor FrenteDerecho = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor AtrasIzquierdo = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor AtrasDerecho = hardwareMap.get(DcMotor.class, "rightBack");

        //Declarar variables de movimiento de robot
        double drive;   //mover hacia delante
        double turn;    //girar
        double strafe;  //ir en diagonal

        //Declarar variables de poder de motores
        double FIPower;
        double FDPower;
        double AIPower;
        double ADPower;

        //Inverting one of the sides for a linear drive
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        FrenteDerecho.setDirection(DcMotorSimple.Direction.FORWARD);
        AtrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);

        //Usar encoders en los motores del chassis
        FrenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Inicializar servo del intake
        CRServo servo1;
        servo1 = hardwareMap.get(CRServo.class, "serv intake");
        servo1.setDirection(CRServo.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        double powerI;
        double powerS;
        double IServopower;
        powerI = 0.8;
        IServopower = 0;
        powerS = 0;
        Intake.setPower(powerI);
        ShooterR.setPower(powerS);
        ShooterL.setPower(powerS);
        servo1.setPower(IServopower);

        while (opModeIsActive()) {

            //configurar valores del joystick del control
            drive = gamepad1.left_stick_y * -0.8; //mover el robot
            turn = gamepad1.right_stick_x; //girar el robot
            strafe = gamepad1.left_stick_x; //mover el robot en diagonal

            //Modo lento
            if(gamepad1.a){ //Presionar boton "a"
                drive = gamepad1.left_stick_y * -0.2;       //limitar motor a 0.2 de potencia
                strafe = gamepad1.left_stick_x * -0.2;
            }else{drive = gamepad1.left_stick_y * -0.8;
                strafe = gamepad1.left_stick_x * -0.8;}    //si no se presiona el boton, dejar el motor a 0.8 de potencia



            //Give initial power to motors
            if (gamepad1.b) {
                IServopower = 0.8;
            } else {
                IServopower = 0;
            }

            if (gamepad1.right_bumper) {
                powerS = 0.8;
            } else {
                powerS = 0;
            }

            if (gamepad1.y) {
                Intake.setDirection(DcMotorSimple.Direction.FORWARD);
                servo1.setDirection(CRServo.Direction.REVERSE);
            } else {
                Intake.setDirection(DcMotorSimple.Direction.REVERSE);
                servo1.setDirection(CRServo.Direction.FORWARD);
            }

            //Declarar variables de poder de motores
            FIPower = drive + turn + strafe; //positivo hacia enfrente
            FDPower = drive - turn - strafe; //opuesto que frente izquierdo
            AIPower = drive + turn - strafe; //invertimos para ir hacia atras
            ADPower = drive - turn + strafe; //opuesto que atras izquierdo


            //Dar poder a los motores
            Intake.setPower(powerI);
            ShooterR.setPower(powerS);
            ShooterL.setPower(powerS);
            servo1.setPower(IServopower);
            FrenteIzquierdo.setPower(FIPower);
            FrenteDerecho.setPower(FDPower);
            AtrasIzquierdo.setPower(AIPower);
            AtrasDerecho.setPower(ADPower);

            telemetry.addData("Velocidad Intake: ", powerI);
            telemetry.addData("Velocidad Shooter: ", powerS);
            telemetry.addData("Velocidad servos: ", IServopower);
            telemetry.update();
        }
    }
}
