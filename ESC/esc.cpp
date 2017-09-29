#include "mbed.h"
#include "esc.h"

/** Initializes the PwmOut for minimum throttle (1000us).
 * @param pwmPinOut is the pin connected to the ESC.
 * @param period is the PWM period in ms (default is 20ms).
 */
ESC::ESC(const PinName pwmPinOut_dw_rx, const PinName pwmPinOut_dw_lx,
         const PinName pwmPinOut_dw_rx_lx, const PinName pwmPinOut_dw_rx_rx,
         const int period): esc_dw_rx(pwmPinOut_dw_rx),
         esc_dw_lx(pwmPinOut_dw_lx), esc_up_lx(pwmPinOut_dw_rx_lx),
         esc_up_rx(pwmPinOut_dw_rx_rx), period(period)
{
    //set every esc to the minimum throttle
    esc_dw_rx.period_ms(period);
    esc_dw_rx.pulsewidth_us(1100);
    esc_dw_lx.period_ms(period);
    esc_dw_lx.pulsewidth_us(1100);
    esc_up_lx.period_ms(period);
    esc_up_lx.pulsewidth_us(1100);
    esc_up_rx.period_ms(period);
    esc_up_rx.pulsewidth_us(1100);
}

/** Sets the throttle value
 * @param t in in the range [0.0;1.0]
 * @return true if throttle value is in range; false otherwise.
 */
bool ESC::setThrottle(const float t)
{
    if (t>=0.0&&t<=1.0) {                           // qualify range, 0-1    
        esc_dw_rx.pulsewidth_us(1000.0*t + 1100);   // map to range, 1.1-2.1 ms
        esc_dw_lx.pulsewidth_us(1000.0*t + 1100);   // (1100-2100us)
        esc_up_lx.pulsewidth_us(1000.0*t + 1100);
        esc_up_rx.pulsewidth_us(1000.0*t + 1100);
        return true;
    }
    return false;
}

bool ESC::setThrottle (const float t_dw_rx, const float t_dw_lx, const float t_up_lx, const float t_up_rx) //non messo nella tesi perchè le correzioni sono a cazzo di cane, stiamo un attimo a inserirlo volendo
{
    if ((t_dw_rx>=0.0&&t_dw_rx<=1.0)&&(t_dw_lx>=0.0&&t_dw_lx<=1.0)&&(t_up_lx>=0.0&&t_up_lx<=1.0)&&(t_up_rx>=0.0&&t_up_rx<=1.0)) {       // qualify range, 0-1    
        esc_dw_rx.pulsewidth_us((1000.0*t_dw_rx + 1100)*0.95);      // map to range, 1.1-2.1 ms (1100-2100us)
        esc_dw_lx.pulsewidth_us((1000.0*t_dw_lx + 1100)*0.85*0.95);
        esc_up_lx.pulsewidth_us((1000.0*t_up_lx + 1100)*0.85);
        esc_up_rx.pulsewidth_us((1000.0*t_up_rx + 1100)*0.95);
        return true;
    }
    return false;
}
bool ESC::setThrottlePure (const float t_dw_rx, const float t_dw_lx
                       const float t_up_lx, const float t_up_rx)
{
    if ((t_dw_rx>=0.0&&t_dw_rx<=1.0)&&(t_dw_lx>=0.0&&t_dw_lx<=1.0)
     &&(t_up_lx>=0.0&&t_up_lx<=1.0)&&(t_up_rx>=0.0&&t_up_rx<=1.0)) {
        // qualify range, 0-1    
        esc_dw_rx.pulsewidth_us((1000.0*t_dw_rx + 1100));//map 2 range,1.1-2.1ms 
        esc_dw_lx.pulsewidth_us((1000.0*t_dw_lx + 1100));
        esc_up_lx.pulsewidth_us((1000.0*t_up_lx + 1100));
        esc_up_rx.pulsewidth_us((1000.0*t_up_rx + 1100));
        return true;
    }
    return false;
}

