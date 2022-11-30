#include"kalman.h"
#include <fstream>

int main()
{
    Kalman k(1,200); //arg1：Q;arg2:R

    k.get_zk();

    for(int i = 0;i<k.get_len_zk();i++)
    {
        k.predict_equation();
        k.update_equation(i);
        k.get_result();
    }
    k.save_result();
    cout<<"the data saved"<<endl;
}

void Kalman:: predict_equation()
{
    _xk = _Fk * _xk + _Bk * _uk;
    _Pk = _Fk * _Pk * _Fk + _Q;
}
void Kalman::update_equation(int& i)
{
    _K = _Pk * _hk / (_hk * _Pk * _hk + _R);
    _xk = _xk + _K * (_zk[i] - _hk * _xk); //算出最后的最优估计
    _Pk = _Pk - _K * _hk * _Pk;
}

void Kalman::get_result()
{
    x_list.push_back(_xk);
}

void Kalman::get_zk()
{
    ifstream f;
    f.open("1.4-14.txt",ios::in);
    if(!f) {
        cerr<<"出错啦~这个文件出错啦！"<<endl;
    }
    float s;//我创建的变量，存储数据用的
    while(f>>s)
    {//输入文件流
        _zk.push_back(s);
    }
    _xk = _zk[0];
}

int Kalman::get_len_zk()
{
    int num;
    num = _zk.size();
    return num;
}

void Kalman::save_result()
{
    ofstream f;
    f.open("filtered.txt",ios_base::out);
    for(int i=0;i<x_list.size();i++)
    {
        f<<x_list[i]<<endl;
    }
    f.close();
}

