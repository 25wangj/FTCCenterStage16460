package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public abstract class AbstractMecanumDrivetrain extends Drivetrain {
    private DcMotorEx fr;
    private DcMotorEx fl;
    private DcMotorEx br;
    private DcMotorEx bl;
    public AbstractMecanumDrivetrain(String frName, String flName, String brName, String blName, LinearOpMode opMode, Localizer localizer, boolean auto) {
        super(opMode, localizer, auto);
        this.fr = opMode.hardwareMap.get(DcMotorEx.class, frName);
        this.fl = opMode.hardwareMap.get(DcMotorEx.class, flName);
        this.br = opMode.hardwareMap.get(DcMotorEx.class, brName);
        this.bl = opMode.hardwareMap.get(DcMotorEx.class, blName);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setPowers(double x, double y, double t) {
        double d = max(abs(x) + abs(y) + abs(t), 1);
        fr.setPower((x + y + t) / d);
        fl.setPower((x - y - t) / d);
        br.setPower((x - y + t) / d);
        bl.setPower((x + y - t) / d);
    }
}
