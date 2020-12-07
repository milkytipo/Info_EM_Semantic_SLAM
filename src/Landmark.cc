/**
* This class will save the landmark (object) information 
* Designed for semantic object.
*/

#include "Landmark.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int Landmark::nNextId=0;
mutex MapPoLandmarkint::mGlobalMutex;

Landmark::Landmark(const size_t id):mLandmarkId(id)
{
    mLastCluserSize = 0;
};


void Landmark::AddLandmarkPoint(MapPoint* pMapPoint){
    mvlmCluster.push_back(pMapPoint);
}

void Landmark::AddCurrentLandmarkPoint(MapPoint* pMapPoint){
    mvlmClusterCurrentFrame.push_back(pMapPoint);
}


bool Landmark::isGoodObservation(){

    size_t N = mvlmClusterCurrentFrame.size();
    float_t infoTemp = 0;
    for(vector<MapPoint*>::iterator it=mvlmClusterCurrentFrame.begin();it!=mvlmClusterCurrentFrame.end();it++){
        infoTemp += (*it)->mEntropy;
    }

    //The average info should not be changed too large 
    size_t N_2 = mvlmCluster.size();
    if (N_2 > 0){
        if( abs( mInfoSum/N_2 - infoTemp/N ) > mInfoSum/(N_2*3.0)){ 
            mvlmClusterCurrentFrame.clear();
            mGoodObservation = false;
        }else{
            mvlmCluster.insert(mvlmCluster.end(), mvlmClusterCurrentFrame.begin(), mvlmClusterCurrentFrame.end())
            mvlmClusterCurrentFrame.clear();
            mGoodObservation = true;
        }
    }

    return mGoodObservation;
}

void Landmark::CalculateInfo(){

    size_t N = mvlmCluster.size();
    
    for(vector<MapPoint*>::iterator it=mvlmCluster.begin();it!=mvlmCluster.end();it++){
        mInfoSum +=(*it)->mEntropy;
    }
    
    mLastCluserSize = N;
}


} //namespace ORB_SLAM