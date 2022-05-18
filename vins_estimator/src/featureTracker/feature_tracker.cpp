/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "feature_tracker.h"

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;

    // why 构造函数中 初始化 kdtree，depthcloud指针 
    KdTree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    depthCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // endl
}

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// why 2020-12-10 将vector<cv::Point2f> 转成 vector<pair<double, cv::Point2f>> 并初始化深度值为 0
vector<pair<double, cv::Point2f>> FeatureTracker::transPtsType(vector<cv::Point2f> &pts)
{
    vector<pair<double, cv::Point2f>> outPts;
    outPts.clear();
    for(size_t i = 0; i < pts.size(); i++){
        outPts.push_back(make_pair(0, pts[i]));
    }
    return outPts;
}


// why: 使用激光点云进行特征点数据关联的 trackImage ===================================================================
map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> 
FeatureTracker::trackLImage(double _cur_time, const cv::Mat &_img, const pcl::PointCloud<pcl::PointXYZ>& _lidar_points, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;

    cur_lidar_ = _lidar_points; // why 

    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();
    cur_pts_pair_.clear();
    cur_lidar_pts_.clear();

    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        if(hasPrediction)  // 利用恒速模型对路标点坐标进行了预测
        {
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)  //从后一帧图像，计算前一帧图像的points，进行额外筛选，提升鲁棒性
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;

        reduceVector(cur_pts, status);  // 对不满足条件的points，进行剔除
        reduceVector(prev_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        // setMaskForL();
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)  // 如果跟踪的points数目未达到设定的数目MAX_CNT，额外提取角点，此处为Shi Tomasi点
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)  //将新提取的点保存
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);  // TODO(tzhang):超过int最大范围导致溢出，怎么办？ Bug
            track_cnt.push_back(1);
        }
        // printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);  //去畸变
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);  //计算在归一化相机坐标系下的速度

    if(!_img1.empty() && stereo_cam)  //双目情形，右图跟踪左图上的points，与前面的前后帧处理方法类似
    {
        ids_right.clear();

        cur_right_pts.clear();
        cur_un_right_pts.clear();

        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();

        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status); 
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }

    // why 2020-12-10 将 cur_pts 放入到 cur_pts_pair, cur_right_pts 放入到 cur_right_pts_pair
    cur_pts_pair_ = transPtsType(cur_pts);
    cur_un_pts_pair_ = transPtsType(cur_un_pts);
    cur_right_pts_pair_ = transPtsType(cur_right_pts);
    cur_un_right_pts_pair_ = transPtsType(cur_un_right_pts);
    // end


    // kdtree 匹配  --------------------------------------------------------------
    depthCLoudProj_kdtree(_lidar_points);
    dataAssociatCamLidar(cur_pts_pair_);            // 对去畸变之后的点进行数据关联得到 正确的深度值
    // dataAssociatCamLidar(cur_right_pts_pair_);      // 对去畸变之后的右目点进行数据关联得到 正确的深度值

    // std::cout << "_lidar_points size is = " << _lidar_points.size() << std::endl; // why
    if(SHOW_TRACK)  //显示跟踪的路标点，就是rosviz双目图像上那些点
        drawTrack(cur_img, rightImg, ids, cur_pts_pair_, cur_right_pts, _lidar_points, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;  // feature_id  camera_id  x, y, z, p_u, p_v, velocity_x, velocity_y, depth; 
                            //（即特征点编号、相机编号（0表示左相机，1表示右相机）、每个特征点参数（归一化相机坐标系坐标、图像坐标（矫正后）、归一化相机坐标系下路标点速度）
    
    int count_cur_pts_pairfirst = 0; // why for debug
    for (size_t i = 0; i < ids.size(); i++)
    // for (size_t i = 0; i < cur_pts_.second.size(); i++)  // why 2020.11.3 此处是造成后面 addFeatureCheckParallax中id_pts.second[1].first == 1 断言失败的原因
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts_pair_[i].second.x;
        y = cur_un_pts_pair_[i].second.y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts_pair_[i].second.x;
        p_v = cur_pts_pair_[i].second.y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        double depth = cur_pts_pair_[i].first; // why  -----------------------------------------------------
        if(depth > 0) // why for debug
        {
            count_cur_pts_pairfirst++;
            // std::cout << "00 left feature_id is " << feature_id << std::endl;
        }
            
        // cout << "cur_pts_pair_[i].first is ------------- " << depth << endl; // for debug
        Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);

    }
    // std::cout << "3 -------------  count_cur_pts_pairfirst  " << count_cur_pts_pairfirst << std::endl; // why for debug

    int count_cur_right_pts_pairfirst = 0; // why for debug
    // 双目情况 why注释 // 此处深度值错误 
    if (!_img1.empty() && stereo_cam) 
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts_pair_[i].second.x;
            y = cur_un_right_pts_pair_[i].second.y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts_pair_[i].second.x;
            p_v = cur_right_pts_pair_[i].second.y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            // double depth = cur_right_pts_pair_[i].first; // why  
            // double depth = 0; // why  
            double depth = 0;
            if(featureFrame[feature_id][0].second[7] != 0)     // why 2021.1.12 TODO：判断左目是关联深度，若关联，则此id的右目不进行赋值深度
                depth = featureFrame[feature_id][0].second[7];    // why 2020-12-07 目前有目不关联深度值，只使用左目和激光雷达关联的深度值， 如果没有则使用 双目三角化深度
                                                                // 2021.1.12 此处的左目激光雷达关联的深度值， 被右目这里覆盖了？？
            
            if(featureFrame[feature_id][0].second[7] > 0) // why for debug
            {
                count_cur_right_pts_pairfirst++;
                // std::cout << "02 left  feature_id is " << feature_id << std::endl;
            }
            Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
            xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity_depth);
 
        }
    }
    // std::cout << "4 -------------  count_cur_right_pts_pairfirst " << count_cur_right_pts_pairfirst << std::endl; // why for debug


    cur_pts_pair_.clear();
    cur_un_pts_pair_.clear();
    cur_right_pts_pair_.clear();
    cur_un_right_pts_pair_.clear();
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
    //（即特征点编号、相机编号（0表示左相机，1表示右相机）、每个特征点参数（归一化相机坐标系坐标、图像坐标（矫正后）、归一化相机坐标系下路标点速度）
}


void FeatureTracker::depthCLoudProj_kdtree(const pcl::PointCloud<pcl::PointXYZ>& curlidarpoints)
{
    depthCloud_->clear();
    depthCloudNum_ = curlidarpoints.size();
    pcl::copyPointCloud(curlidarpoints, *depthCloud_);

    if(depthCloudNum_ > 10){
        for(size_t i = 0; i < depthCloudNum_; i++){
            if(curlidarpoints[i].y < 0){
                depthCloud_->points[i].intensity = curlidarpoints[i].z;
                depthCloud_->points[i].x = curlidarpoints[i].x * 10 / curlidarpoints[i].z;
                depthCloud_->points[i].y = curlidarpoints[i].y * 10 / curlidarpoints[i].z;
                depthCloud_->points[i].z =  10;
            }
        }
        KdTree_->setInputCloud(depthCloud_);
    }
}

void FeatureTracker::dataAssociatCamLidar( vector< pair<double, cv::Point2f> > &curpts)
{
    pcl::PointXYZI ips;
    pcl::PointXYZHSV ipr;
    int cout_iprs = 0;
    // std::cout << "curpts.first.size is " << curpts.first.size() << "   " << curpts.second.size() << std::endl;
    for(unsigned int i = 0; i < curpts.size(); i++)
    {   
        // 归一化公式
        // point.u = -(featuresLast[featureCount].x - kImage[2]) / kImage[0];
        // point.v = -(featuresLast[featureCount].y - kImage[5]) / kImage[4];
        // 小觅
        ipr.x = (curpts[i].second.x - 327.12) / 359.32;
        ipr.y = (curpts[i].second.y - 236.01) / 359.72;
        // kitti
        // ipr.x = (it.x - 607.2) / 718.8;
        // ipr.y = (it.y - 185.2) / 718.8;

        ips.x = 10 * ipr.x;
        ips.y = 10 * ipr.y;
        ips.z = 10;

        if( depthCloudNum_ > 10 ){
            KdTree_->nearestKSearch(ips, 3, pointSearchInd_, pointSearchSqrDis_);

            double minDepth, maxDepth;
            // 如果搜寻的点为三个，且距离小于阈值, 原 0.01
            if (pointSearchSqrDis_[0] < 0.03 && pointSearchInd_.size() == 3) 
            {
                // 查找的依据是投影后的点，而我们现在要找相对应的深度图原有的点，intensity存储的是原有的z值
                pcl::PointXYZI depthPoint = depthCloud_->points[pointSearchInd_[0]];
                double x1 = depthPoint.x * depthPoint.intensity / 10;
                double y1 = depthPoint.y * depthPoint.intensity / 10;
                double z1 = depthPoint.intensity;
                minDepth = z1; 
                maxDepth = z1;

                depthPoint = depthCloud_->points[pointSearchInd_[1]];
                double x2 = depthPoint.x * depthPoint.intensity / 10;
                double y2 = depthPoint.y * depthPoint.intensity / 10;
                double z2 = depthPoint.intensity;
                minDepth = (z2 < minDepth)? z2 : minDepth; // 如果z2小于 最小深度 则minDepth = z2
                maxDepth = (z2 > maxDepth)? z2 : maxDepth; // 如果z2大于 最大深度 则maxDepth = z2

                depthPoint = depthCloud_->points[pointSearchInd_[2]];
                double x3 = depthPoint.x * depthPoint.intensity / 10;
                double y3 = depthPoint.y * depthPoint.intensity / 10;
                double z3 = depthPoint.intensity;
                minDepth = (z3 < minDepth)? z3 : minDepth;
                maxDepth = (z3 > maxDepth)? z3 : maxDepth;
                // 以上是从三个点中找最大最小深度值

                // 目前只知道该特征点在相机坐标系下的归一化坐标[u,v]（即[X/Z,Y/Z,1]）
                // 通过计算ipr.s获得对应于该特征点的深度值，即系数Z，则Z*u和Z*v就可以获得该特征点在相机坐标
                double u = ipr.x;
                double v = ipr.y;
                // 对应论文公式（10）,ipr.s对应的就是深度值
                ipr.s = (x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1) 
                    / (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2 + u*y1*z2 - u*y2*z1
                    - v*x1*z2 + v*x2*z1 - u*y1*z3 + u*y3*z1 + v*x1*z3 - v*x3*z1 + u*y2*z3 
                    - u*y3*z2 - v*x2*z3 + v*x3*z2);  

                // 当三点深度差距过大说明投影附近存在遮挡关系或者物体边缘，丢弃
                if(maxDepth - minDepth > 2){
                    ipr.s = 0;
                }else if(ipr.s - maxDepth > 0.2){
                    ipr.s = maxDepth;
                }else if(ipr.s - minDepth < -0.2){
                    ipr.s = minDepth;
                }
            }
            else
                ipr.s = 0;
        }else
            ipr.s = 0;


        if(ipr.s > 0){

            // std::cout << "ipr.s is " << ipr.s << std::endl;
            cout_iprs++;
            curpts[i].first= ipr.s;
        }
    } 
    // std::cout << "1 --------- cout_iprs is = " << cout_iprs << std::endl; // why for debug
}


void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                            //    const pair<vector<double>, vector<cv::Point2f> > &curLeftPts,
                               const vector<pair<double, cv::Point2f> > &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               const pcl::PointCloud<pcl::PointXYZ> &lidarpoints,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    // cout << " track_cnt is " << track_cnt.size() << endl; // why for debug

    int count_curLefPts_d = 0; // why for debug
    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        if(curLeftPts[j].first > 0.1){ // 显示激光雷达关联的特征点
            // std::cout << "curLeftIds[i] is  = " << curLeftIds[j] << "      curLeftPts[j].first is = " << curLeftPts[j].first << std::endl;
            // cv::circle(imTrack, curLeftPts[j].second, 2, cv::Scalar(0, 255 * curLeftPts[j].first / 8, 255 * curLeftPts[j].first / 12), 2);
            cv::circle(imTrack, curLeftPts[j].second, 2, cv::Scalar(0, 255, 0), 2);

            count_curLefPts_d++; // why for debug
        }
        else // 显示左目特征点
            cv::circle(imTrack, curLeftPts[j].second, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    // std::cout << "2 ------------------count_curLefPts_d.size is = " << count_curLefPts_d << std::endl; // why for debug
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 255), 2);
            // cv::Point2f leftPt = curLeftPtsTrackRight[i];
            // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            // cv::arrowedLine(imTrack, curLeftPts[i].second, mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    // why
    bool showlidar = true;
    if(showlidar){
        // 可视化点云投影
        // 3x4 投影矩阵，用于从矫正后的0号相机坐标系 投影到 X号相机的图像平面。
        cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
        // 小觅
        P_rect_00.at<double>(0,0) = 3.5932730e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 3.27129e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
        P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 3.59721984e+02; P_rect_00.at<double>(1,2) = 2.360148e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
        P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

        // kitti
        // P_rect_00.at<double>(0,0) = 7.188560000000e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.071928000000e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
        // P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.188560000000e+02; P_rect_00.at<double>(1,2) = 1.852157000000e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
        // P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);

        for(int i = 0; i < lidarpoints.size(); i++)
        {
            if(lidarpoints[i].y < 0){
                X.at<double>(0, 0) = lidarpoints[i].x;
                X.at<double>(1, 0) = lidarpoints[i].y;
                X.at<double>(2, 0) = lidarpoints[i].z;
                X.at<double>(3, 0) = 1;

                // 投影激光点到相机
                Y = P_rect_00 * X;

                cv::Point pt;
                pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
                pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

                double color = sqrt(lidarpoints[i].x * lidarpoints[i].x + lidarpoints[i].y * lidarpoints[i].y + lidarpoints[i].z * lidarpoints[i].z);
                // double color = 10;
                cv::circle(imTrack, pt, 1.6, cv::Scalar(color * 40, color * 5, 0), -1);
            }
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

// end ================================================================================================
 

map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    if (EQUALIZE)   // 直方图均衡化 2020-1-3 why 取消注释
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }

    cur_pts.clear();

    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cout << "hasPrediction----------------------------------------------- " << endl;
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        rejectWithF(); // 通过计算F矩阵排除outlier .. 2020-1-3 why 取消注释

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);

            /******why修改，增加边特征选点,好像没正作用 */
                // ExtractEdgeFeature(n_pts, MAX_CNT - cur_pts.size());
            /*** */
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
    {
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        // showUndistortion("undis");
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        double depth = 0; // why  ---------------
        Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            double depth = 0; // why  ---------------
            Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
            xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

// why添加.边特征
/**
 * @brief  检测边特征
 * @note   
 * @param  &total_pts:  传入总的特征点数
 * @param  NeedNum: 
 * @retval None
 */
void FeatureTracker::ExtractEdgeFeature(std::vector<cv::Point2f> &total_pts, int NeedNum)
{
    cv::Mat cur_Mask = mask.clone();
    cv::Mat img = cur_img.clone();
    cv::Mat gradx = cv::Mat::zeros(cur_img.rows, cur_img.cols, CV_32F); // x方向梯度
    cv::Mat grady = cv::Mat::zeros(cur_img.rows, cur_img.cols, CV_32F); // y方向梯度
    cv::Mat mag = cv::Mat::zeros(cur_img.rows, cur_img.cols, CV_32F);

    cv::GaussianBlur(cur_img, img, cv::Size(3, 3), 0, 0); // 高斯滤波
    cv::Scharr(img, gradx, CV_32F, 1, 0, 1/32.0); // Scharr滤波器，计算x、y的图像梯度，
    cv::Scharr(img, grady, CV_32F, 0, 1, 1/32.0);
    cv::magnitude(gradx, grady, mag); // 计算二维矢量的幅值

    cv::Mat canny;
    cv::Canny(img, canny, 30, 50); // Canny边缘检测

    if(total_pts.size() != 0 )
    {
        for(int k=0; k < total_pts.size(); ++k)
        {
            cv::circle(cur_Mask, cv::Point2f((float)total_pts[k].x, (float)total_pts[k].y), MIN_DIST, 0, -1);
        }
    }
    
    for(int x=1; x + MIN_DIST < cur_img.cols - 1; x = x + MIN_DIST)
    {
        for(int y = 1; y + MIN_DIST < cur_img.rows - 1; y = y + MIN_DIST)
        {
            float max_grad = 0;
            int maxgrad_x = 0;
            int maxgrad_y = 0;
            float gx = 0;
            float gy = 0;

            float max_grad_2 = 0;
            int maxgrad_x2 = 0;
            int maxgrad_y2 = 0;

            for (int i=0; i<MIN_DIST; i++)
            {
                for(int j=0; j<MIN_DIST; j++)
                {
                    if(cur_Mask.ptr<uchar>(y + i)[x + j] != 255) continue;

                    float temp = mag.ptr<float>(y + i)[x + j]; // 此像素梯度
                    if(temp > max_grad_2)
                    {
                        maxgrad_x2 = x + j;
                        maxgrad_y2 = y + i;
                        max_grad_2 = temp;
                    }

                    if(canny.ptr<uchar>(y + i)[x + j] == 0) continue;
                    if(cur_Mask.ptr<uchar>(y + i)[x + j] != 255) continue;

                    if (temp > max_grad)
                    {
                        maxgrad_x = x + j;
                        maxgrad_y = y + i;
                        max_grad =temp;
                        gx = gradx.ptr<float>(maxgrad_y)[maxgrad_x];
                        gy = grady.ptr<float>(maxgrad_y)[maxgrad_x];
                    }
                }
            }
            float edge_threshold = 0.1;
            {
                if(max_grad > edge_threshold)
                {
                    total_pts.push_back(cv::Point2f((float)maxgrad_x, (float)maxgrad_y));
                    cv::circle(cur_Mask, cv::Point2f((float)maxgrad_x, (float)maxgrad_y), MIN_DIST, 0, -1);
                }
                // else if(max_grad_2 > 0.1)
                // {
                //     total_pts.push_back(cv::Point2f((float)maxgrad_x2, (float)maxgrad_y2));
                //     cv::circle(cur_Mask, cv::Point2f((float)maxgrad_x2, (float)maxgrad_y2), MIN_DIST, 0, -1);
                // }
                
            }
        }
    }
} 
// end

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]); // 返回相机类型
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    cv::imshow(name, undistortedImg);
    cv::waitKey(1);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 255), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}