#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "mbed.h"

class Ultrasonic
{
    public:
        /** Initiates the class with the specified trigger pin, echo pin, update speed and timeout
         */
        Ultrasonic(PinName trigPin, PinName echoPin, float updateSpeed, float timeout);
        
        /** Returns the last measured distance*/
        float getCurrentDistance(void);
        
        /** Pauses measuring the distance*/
        void pauseUpdates(void);
        
        /** Starts mesuring the distance
         * Call this once, at the beginning
         */
        void startUpdates(void);
      
    private:
        DigitalOut _trig;
        InterruptIn _echo;
        Timer _t; //timer to count the length of the impulse
        Timeout _tout; //counts the time passed without receiving any echo
        float _distance;
        float _updateSpeed;
        int start;
        int end;
        float _timeout;
        
        /** Called everytime the echo rises
         * It starts counting
         */
        void _startT(void);
        
        /** Called everytime the echo falls
         * It ends counting and calculates the time passed between rise and fall.
         * Then, it calcualtes the distance traveled by the ultrasonic impulse
         */
        void _updateDist(void);
        
        /** Run with _updateSpeed frequency, or at least when the timeout occurs
         * It launches a 10us impulse from the trigger
         * It also attaches to the echo the rise and fall methods.
         * Finally it attaches itself to the timeout, so when the timeout occurs
         * without receiving the echo, it runs again, sending another signal etc
         */
        void _startTrig(void);
};
#endif