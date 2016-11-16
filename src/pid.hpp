#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previousTime(ros::Time::now())
	, m_lastOutput(0)
	, m_pubVelocity()
	, m_pubFilterVelocity()
	, m_p(0)
	, m_i(0)
	, m_d(0)
    {
        ros::NodeHandle nh;
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }
    
    float getIntegral(void)
    {
        return m_integral;
    }

    void setPID(float P, float I, float D)
    {
	m_kp = P < 0 ? m_kp : P;
	m_ki = I < 0 ? m_ki : I;
	m_kd = D < 0 ? m_kd : D;
    }
    
    float getP(void)
    {
        return m_p;
    }
    
    float getI(void)
    {
        return m_i;
    }
    
    float getD(void)
    {
        return m_d;
    }
    
    float ki() const
    {
        return m_ki;
    }

    float kd() const
    {
        return m_kd;
    }

    float kp() const
    {
        return m_kp;
    }

    float update(float value, float targetValue, float voltage)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        error=(error<10e10 && error>-10e10)?error:0;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
	float p;
	if(voltage)
            p = (m_kp + (4.3 - voltage) * m_kp / 2.0) * error;
	else
	    p = m_kp * error;
        float d = 0;
        if (dt > 0)
        {
		d = m_kd * simple_low_pass((error - m_previousError) / dt, dt);//(error - m_previousError) / dt;
	    //else
		//d = m_kd * (error - m_previousError) / dt;
        }
        float i = m_ki * m_integral;
        float output = p + d + i;
        m_p=p;m_i=i;m_d=d;
        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

    float simple_low_pass(float input, float dt)
    {
	std_msgs::Float32 Velocity;
	Velocity.data = input;
        float CUTOFF = 2.0;
        float alpha = 1.0 / (1.0 / (2.0 * 3.1415926 * CUTOFF * dt) + 1.0);
        float output = m_lastOutput + alpha * (input - m_lastOutput);
	if(input < 20 && input > -20)
	    m_lastOutput = output;
	//else
	//    m_lastOutput = 0;
	std_msgs::Float32 FilterVelocity;
	FilterVelocity.data = m_lastOutput;
	return m_lastOutput;
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    float m_lastOutput;
    float m_p;
    float m_i;
    float m_d;
    ros::Time m_previousTime;
    ros::Publisher m_pubVelocity;
    ros::Publisher m_pubFilterVelocity;
};
