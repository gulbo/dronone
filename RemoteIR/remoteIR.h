#ifndef REMOTE_IR_H_
#define REMOTE_IR_H_

class RemoteIR {
public:

    typedef enum {
        UNKNOWN,
        NEC,
        NEC_REPEAT,
        AEHA,
        AEHA_REPEAT,
        SONY
    } Format;

    static const int TUS_NEC = 562;
    static const int TUS_AEHA = 425;
    static const int TUS_SONY = 600;

private:
    RemoteIR();
};

#endif
