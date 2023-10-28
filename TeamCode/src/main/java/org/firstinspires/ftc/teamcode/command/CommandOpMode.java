package org.firstinspires.ftc.teamcode.command;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public abstract class CommandOpMode extends LinearOpMode {
    protected Scheduler scheduler;
    public abstract void initOpMode();
    public void waitOpMode() {}
    public void startOpMode() {}
    @Override
    public void runOpMode() {
        scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
        }
        startOpMode();
        while (opModeIsActive()) {
            scheduler.run(true);
            telemetry.update();
        }
    }
    public void register(Subsystem... subsystems) {
        scheduler.register(subsystems);
    }
}
