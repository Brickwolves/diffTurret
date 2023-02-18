package org.firstinspires.ftc.teamcode.Hardware;

public class V4BUpdater extends Thread{
    Scoring scorer;
    double target;
    boolean runPID = false;
    boolean stop = false;
    public V4BUpdater(Scoring scorer){
        this.scorer = scorer;
    }
    public void setTarget(double target){
        this.target = target;
    }

    @Override
    public void run() {
        while(!stop && !super.isInterrupted()){
            if(runPID){
                scorer.v4b(target);
            }
        }
    }
    public void doRun(boolean run){
        runPID = run;
    }
    public void exit(){
        stop = true;
        super.interrupt();
    }
}
