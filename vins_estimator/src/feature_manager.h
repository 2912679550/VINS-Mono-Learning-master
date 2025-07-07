#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

/**
* @class FeaturePerFrame
* @brief 特征类
* detailed 
*/
class FeaturePerFrame
{
  public:
    //_point:[x,y,z,u,v,vx,vy]
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;     // 归一化坐标
    Vector2d uv;        // 像素坐标
    Vector2d velocity;  // 像素速度
    double z;           // ？ 这个z是深度吗
    bool is_used;
    double parallax;    // 视差
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

/**
* @class FeaturePerId
* @brief 某feature_id下的所有FeaturePerFrame
* detailed 
*/
class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)  //以feature_id为索引，并保存了出现该角点的第一帧的id
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);
    // * 特征点添加与视差检查
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    
    // * 深度管理
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void setDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void clearDepth(const VectorXd &x);
    
    // * 滑窗操作与管理
    void removeFront(int frame_count);  // 移除最新帧
    void removeBack();                  // 移除最旧帧
    // 移除帧时的深度变换
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    
    // * 数据清理
    void removeFailures();  // 清除三角化失败的点
    void removeOutlier();   // 移除外点
    void clearState();      // 清除所有状态
    
    // * 数据查询
    int getFeatureCount();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    
    void setRic(Matrix3d _ric[]);
    void debugShow();

    //void updateDepth(const VectorXd &x);
    // * 核心数据结构
    list<FeaturePerId> feature;// 通过FeatureManager可以得到滑动窗口内所有的角点信息
    int last_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif