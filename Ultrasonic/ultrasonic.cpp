#include "ultrasonic.h"

/** Initiates the class with the specified trigger pin, echo pin, update speed and timeout*/
Ultrasonic::Ultrasonic(PinName trigPin, PinName echoPin, float updateSpeed, float timeout):_trig(trigPin), _echo(echoPin)
{
    _updateSpeed = updateSpeed;
    _timeout = timeout;
    _t.start();
}

/** Called everytime the echo rises
 * It starts counting
 */
void Ultrasonic::_startT()
{
    if(_t.read()>600) {
        _t.reset ();
    }
    start = _t.read_us ();
}

/** Called everytime the echo falls
 * It ends counting and calculates the time passed between rise and fall.
 * Then, it calcualtes the distance traveled by the ultrasonic impulse
 */
void Ultrasonic::_updateDist()
{
    end = _t.read_us ();
    _distance = (end - start)/58.2;
    _tout.detach();
    _tout.attach(this,&Ultrasonic::_startTrig, _updateSpeed);
}

/** Run with _updateSpeed frequency, or at least when the timeout occurs
 * It launches a 10us impulse from the trigger
 * It also attaches to the echo the rise and fall methods.
 * Finally it attaches itself to the timeout, so when the timeout occurs
 * without receiving the echo, it runs again, sending another signal etc
 */
void Ultrasonic::_startTrig(void)
{
    _tout.detach();
    _trig=1;
    wait_us(10);
    _trig=0;
    _echo.rise(this,&Ultrasonic::_startT);
    _echo.fall(this,&Ultrasonic::_updateDist);
    _echo.enable_irq();
    _tout.attach(this,&Ultrasonic::_startTrig,_timeout);
}

/** Returns the last measured distance*/
float Ultrasonic::getCurrentDistance(void)
{
    return _distance;
}

/** Pauses measuring the distance*/
void Ultrasonic::pauseUpdates(void)
{
    _tout.detach();
    _echo.rise(NULL);
    _echo.fall(NULL);
}

/** Starts mesuring the distance
 * Call this once, at the beginning
 */
void Ultrasonic::startUpdates(void)
{
    _startTrig();
}
