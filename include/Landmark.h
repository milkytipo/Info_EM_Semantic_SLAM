/**
* This class will save the landmark (object) information 
* Designed for semantic object.
*/

#ifndef LANDMARK_H
#define LANDMARK_H

#include"KeyFrame.h"
#include"MapPoint.h"
#include"Frame.h"
#include"Map.h"
#include<cmath>
#include<vector>
#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;
class MapPoint;

class Landmark
{
public:
    Landmark(const size_t id);
    // MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    // MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    void CalculateInfo();
    
    void AddLandmarkPoint(MapPoint* pMapPoint);
    void AddCurrentLandmarkPoint(MapPoint* pMapPoint);


    bool isGoodObservation();
    
public:
    size_t mLandmarkId;

    size_t mLandmarkClassId;

    std::set<MapPoint*> mvlmCluster; 
    std::set<MapPoint*> mvlmClusterCurrentFrame; 


    float_t mProbSport;

    static std::mutex mGlobalMutex;


private:
    float mInfoSum; 

    bool mSportStatus; // 1:dynamic 0:static

    float_t mTransition; // transitimGlobalMutex

    size_t mLastCluserSize;
};

} //namespace ORB_SLAM

#endif // LANDMARK_H
