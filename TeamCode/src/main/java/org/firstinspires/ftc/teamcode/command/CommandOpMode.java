package org.firstinspires.ftc.teamcode.command;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public abstract class CommandOpMode extends LinearOpMode {
    protected Scheduler scheduler;
    private boolean done = false;
    public abstract void initOpMode();
    public void waitOpMode() {}
    public void startOpMode() {}
    public void endOpMode() {}
    @Override
    public void runOpMode() {
        scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
            telemetry.update();
        }
        startOpMode();
        while (opModeIsActive() && !done) {
            telemetry.addData("Loop speed", 1 / scheduler.run(true));
            telemetry.update();
        }
        endOpMode();
    }
    public void register(Subsystem... subsystems) {
        scheduler.register(subsystems);
    }
    public void end() {
        done = true;
    }
}
