package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum(Obsolete)")
public class Mecanum_DriveTrain extends LinearOpMode {
    @Override
    public void runOpMode(){

        telemetry.addData("Estado", "Funcional");
        telemetry.update();

        //Declarar variables de movimiento de robot
        double drive;   //mover hacia delante
        double turn;    //girar
        double strafe;  //ir en diagonal

        //Declarar variables de poder de motores
        double FIPower;
        double FDPower;
        double AIPower;
        double ADPower;

        //Inicializacion de los 4 motores para el chasis
        DcMotor frenteIzquierdo = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frenteDerecho = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor atrasIzquierdo = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor atrasDerecho = hardwareMap.get(DcMotor.class, "rightBack");

        //Invertir motores para girar en el mismo sentido
        frenteDerecho.setDirection(DcMotorSimple.Direction.FORWARD);
        atrasDerecho.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoders
        frenteIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frenteDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasIzquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        atrasDerecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Pausa para reseteo de IMU
        //Deadline imuLimit = new Deadline(500,TimeUnit.MILLISECONDS);

        //Crear e inicializar el objeto para la IMU
        /*IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
         */


        //Iniciar la Driver Station
        waitForStart();

        while(opModeIsActive()){
            //configurar valores del joystick del control
            //turn = gamepad1.right_stick_x; //girar el robot

            turn = gamepad1.right_trigger - gamepad1.left_trigger;


            /*
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double strafeAjustada = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double driveAjustada = ly * Math.cos(heading) + lx * Math.sin(heading);
             */

            //Modo lento
            if(gamepad1.a){ //Presionar boton "a"
                drive = gamepad1.left_stick_y * -0.2;       //limitar motor a 0.2 de potencia
                strafe = gamepad1.left_stick_x * 0.2;
            }else{drive = gamepad1.left_stick_y * -0.8;
                  strafe = gamepad1.left_stick_x * 0.8;}    //si no se presiona el boton, dejar el motor a 0.8 de potencia

            //Declarar variables de poder de motores
            FIPower = Math.min((drive + turn + strafe), 0.8); //positivo hacia enfrente
            FDPower = Math.min((drive - turn - strafe), 0.8); //opuesto que frente izquierdo
            AIPower = Math.min((drive + turn - strafe), 0.8); //invertimos para ir hacia atras
            ADPower = Math.min((drive - turn + strafe), 0.8); //opuesto que atras izquierdo

            /*
            if(imuLimit.hasExpired() && gamepad1.b){
                imu.resetYaw();
                ireset();
            }
             */

            //Dar poder a los motores
            frenteIzquierdo.setPower(FIPower);
            frenteDerecho.setPower(FDPower);
            atrasIzquierdo.setPower(AIPower);
            atrasDerecho.setPower(ADPower);
        }

    }

}

//ly = drive
//lx = strafe
//rx = turn

//33 was here