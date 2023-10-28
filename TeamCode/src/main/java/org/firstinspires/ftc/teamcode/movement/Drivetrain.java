package org.firstinspires.ftc.teamcode.movement;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public abstract class Drivetrain implements Subsystem {
    protected boolean auto;
    protected Localizer localizer;
    public Drivetrain(LinearOpMode opMode, Localizer localizer, boolean auto) {
        this.auto = auto;
        this.localizer = localizer;
    }
    public void setPose(Pose p) {
        localizer.setPose(p);
    }
    public Pose getPose() {
        return localizer.pos();
    }
    public Pose getVel() {
        return localizer.vel();
    }
    @Override
    public void update(double time, boolean active) {
        if (active && auto) {
            localizer.update(time);
        }
    }
}