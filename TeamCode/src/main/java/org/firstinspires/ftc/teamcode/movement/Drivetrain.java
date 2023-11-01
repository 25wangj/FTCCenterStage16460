package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public abstract class Drivetrain implements Subsystem {
    private boolean auto;
    protected Localizer localizer;
    public Drivetrain(boolean auto) {
        this.auto = auto;
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
        if (active && auto && localizer != null) {
            localizer.update(time);
        }
    }
}