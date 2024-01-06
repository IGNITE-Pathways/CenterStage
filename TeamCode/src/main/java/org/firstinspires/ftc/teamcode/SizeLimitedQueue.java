package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
public class SizeLimitedQueue<E>
        extends LinkedList<E> {

    // Variable which store the
    // SizeLimitOfQueue of the queue
    private int SizeLimitOfQueue;

    // Constructor method for initializing
    // the SizeLimitOfQueue variable
    public SizeLimitedQueue(int SizeLimitOfQueue)
    {
        this.SizeLimitOfQueue = SizeLimitOfQueue;
    }

    // Override the method add() available
    // in LinkedList class so that it allow
    // addition  of element in queue till
    // queue size is less than
    // SizeLimitOfQueue otherwise it remove
    // the front element of queue and add
    // new element
    @Override
    public boolean add(E o)
    {

        // If queue size become greater
        // than SizeLimitOfQueue then
        // front of queue will be removed
        while (this.size() == SizeLimitOfQueue) {

            super.remove();
        }
        super.add(o);
        return true;
    }
}
