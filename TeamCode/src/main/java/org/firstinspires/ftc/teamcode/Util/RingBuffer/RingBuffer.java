package org.firstinspires.ftc.teamcode.Util.RingBuffer;

import java.util.ArrayList;

public class RingBuffer {
    Data[] data;
    int size, lastInsert, nextRead;
    long emitTime;

    public RingBuffer(int size) {
        this.size = size;
        data = new Data[size];
        lastInsert = -1;
        emitTime = System.currentTimeMillis();
    }

    public void insert(Data input) {
        lastInsert = (lastInsert + 1) % size;
        data[lastInsert] = input;

        if (nextRead == lastInsert) {
            nextRead = (nextRead + 1) % size;
        }
    }

    public Data[] emit() {
        Data[] out = new Data[size];
        int start = 0;
        while (true) {
            if (data[nextRead] != null) {
                out[start] = data[nextRead];
                start++;
                data[nextRead] = null;
            }
            if (nextRead == lastInsert || lastInsert == -1) {
                break;
            }
            nextRead = (nextRead + 1) % size;
        }
        return out;
    }
}
