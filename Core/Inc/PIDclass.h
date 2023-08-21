//
// Created by Juiklo J on 2023/8/22.
//

#ifndef PROJECT_PIDCLASS_H
#define PROJECT_PIDCLASS_H
typedef struct{
    float _kp,_ki,_kd;
    float _pout,_iout,_dout,;
    float _currenterror,_lasterror;
    float _integral,_maxintegral,_maxlimit;
    float _output,_maxoutput;
}PID;

void initPID(PID* pid,float kp,float ki,float kd,float maxintegral,float maxlimit,float maxoutput)
{
    pid->_kp=kp;
    pid->_ki=ki;
    pid->_kd=kd;
    pid->_pout=0;
    pid->_iout=0;
    pid->_dout=0;
    pid->_output=0;
    pid->_maxintegral=maxintegral;
    pid->_maxlimit=maxlimit;
    pid->_maxoutput=maxoutput;

}

float myabs(float num)
{
    return num>=0?num:-num;
}

void PID_Cal(PID *pid,float expection,float current)
{
    pid->_lasterror=pid->_currenterror;
    pid->_currenterror=expection-current;
    pid->_pout=pid->_kp*pid->_currenterror;
    if(myabs(pid->_currenterror)>pid->_maxlimit){
        pid->_iout+=pid->_ki*pid->_currenterror;
        pid->_iout=pid->_iout<pid->_maxintegral?pid->_iout:pid->_maxlimit;
    }
    else
    {
        pid->_pout=0;
    }
    pid->_dout=pid->_kd*(pid->_currenterror=pid->_lasterror);
    pid->_output=pid->_iout+pid->_pout+pid->_dout;
    pid->_output=pid->_output<pid->_maxoutput?pid->_output:pid->_maxoutput;
}

#endif //PROJECT_PIDCLASS_H
