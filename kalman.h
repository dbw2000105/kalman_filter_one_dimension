//
// Created by dbstg on 22-11-21.
//

#ifndef KALMANFILTER_KALMAN_H
#define KALMANFILTER_KALMAN_H

#include<iostream>
#include<Eigen/Dense>
#include <vector>
using namespace std;

class Kalman
{
private:
    float _xk; //状态量
    float _Pk; //状态量的协方差矩阵
    float _uk; //系统的输入量（控制向量）
    float _Bk; //控制矩阵
    float _Fk; //状态转移矩阵
    float _Q; //过程噪声（协方差矩阵中的）

    float _hk; //传感器读取的数据的单位和尺度有可能与我们要跟踪的状态的单位和尺度不一样，
                        // 我们用矩阵hk来表示传感器的数据
    vector<float> _zk;//传感器观测数据
    vector<float> x_list;
    float _R; //观测噪声
    float _K;

public:
    /*
     * 卡尔曼滤波的构造函数
     */
    Kalman(float Q, float R)
    {
        _Q = Q;
        _R = R;
        _xk = 0;
        _Pk = 10;
        _uk = 0;
        _Bk = 0;
        _Fk = 1;
        _hk= 1;
        _K = 0;
    }

    /* part1：预测方程
     * 新的最优估计是根据上一最优估计预测得到的，并加上已知外部控制量的修正。
　　而新的不确定性由上一不确定性预测得到，并加上外部环境的干扰。
     */
    void predict_equation();

    /* part2:更新方程
     *
     */
    void update_equation(int& i);

    //显示数据
    void get_result();

    //获取观测值
    void get_zk();

    //获得此次滤波数据的个数
    int get_len_zk();

    //保存结果
    void save_result();
};
#endif