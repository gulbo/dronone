#ifndef ESC_H
#define ESC_H
 
class ESC
{    
  private:
    PwmOut esc_dw_rx;
    PwmOut esc_dw_lx;
    PwmOut esc_up_lx;
    PwmOut esc_up_rx;
    int period;
    
  public:
  
    /** Initializes the PwmOut for minimum throttle (1000us).
     * @param pwmPinOut is the pin connected to the ESC.
     * @param period is the PWM period in ms (default is 20ms).
     */
    ESC(const PinName pwmPinOut_dw_rx, const PinName pwmPinOut_dw_lx, const PinName pwmPinOut_dw_rx_lx, const PinName pwmPinOut_dw_rx_rx, const int period=20);
    
    /** Sets the throttle value
     * @param t in in the range [0.0;1.0]
     * @return true if throttle value is in range; false otherwise.
     */
    bool setThrottle (const float t);
    bool setThrottle (const float t_dw_rx, const float t_dw_lx, const float t_up_lx, const float t_up_rx);
    bool setThrottlePure (const float t_dw_rx, const float t_dw_lx, const float t_up_lx, const float t_up_rx);
 };

#endif