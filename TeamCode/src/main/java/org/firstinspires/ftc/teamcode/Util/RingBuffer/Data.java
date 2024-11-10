package org.firstinspires.ftc.teamcode.Util.RingBuffer;

public class Data {
    long stamp = System.currentTimeMillis();
    double value;
    public Data(double value) {
        this.value = value;
    }

    public Data(int value) {
        this.value = value;
    }
}
