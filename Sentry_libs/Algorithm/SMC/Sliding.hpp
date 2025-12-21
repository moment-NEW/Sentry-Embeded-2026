//
// Created by Lenovo on 2024/3/15.
//

#ifndef KOSANN_UAVGIMBAL_SLIDING_HPP
#define KOSANN_UAVGIMBAL_SLIDING_HPP

#include <cmath>


#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLE_PERIOD 0.001
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

typedef struct {

    float tar_now;//��ǰĿ��ֵ
    float tar_last;//��һ��Ŀ��ֵ
    float tar_differential;//Ŀ��ֵһ�׵�
    float tar_differential_last;//��һ��Ŀ��ֵһ�׵�
    float tar_differential_second;//Ŀ��ֵ���׵�

    float pos_get;//��ǰλ��
    float vol_get;//��ǰ�ٶ�

    float p_error;//λ�����
    float v_error;//�ٶ���λ�����һ�׵���

    float p_error_integral;//λ��������
    float v_error_integral;//�ٶ�������
//    float v_error_integral_max; //�����޷�

    float pos_error_eps;   //����
    float vol_error_eps;   //����
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;   //EIsmc����
    float c2;   //EIsmc����

    float p;    //tfsmc������������ p>q
    float q;    //tfsmc������������
    float beta; //tfsmc����������
    float epsilon; //����������
}SlidingParam;

typedef struct {
    float u; //���������
    float s; //��ģ����㴢��

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max; //����޷�
    Rmode flag; //���źͱ����л���δ�õ�
    float limit; //���ͺ���������
}Sliding;


class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //��Ĥ�����趨 EXPONENT,POWER,VELSMC ����
    void SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//��Ĥ�����趨 TFSMC ����
    void SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC �����趨

    void ErrorUpdate(float target, float pos_now, float vol_now); //��Ĥλ��������
    void ErrorUpdate(float target,float vol_now); //��ģ�ٶ�������
    void Clear();
    void Integval_Clear();

    float SmcCalculate(); //��ģ���������㺯��
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //������ȡ

private:


    Sliding smc;
    void OutContinuation(); //���������
    float Signal(float s); //���ź���
    float Sat(float s); //���ͺ���
};

#ifdef __cplusplus
}
#endif

#endif //KOSANN_UAVGIMBAL_SLIDING_HPP
