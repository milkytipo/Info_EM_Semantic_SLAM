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

Landmark::Landmark(){
    mLastCluserSize = 0;
};


void Landmark::AddLandmarkPoint(MapPoint* pMapPoint){
    mvlmClusterCurrentFrame.push_back(pMapPoint);
}

bool Landmark::isGoodObservation(){

    size_t N = mvlmClusterCurrentFrame.size();
    float_t infoTemp = 0;
    for(int i=0; i<N; i++){
        infoTemp += mvlmClusterCurrentFrame[i]->mEntropy;
    }

    //The average info should not be changed too large 
    size_t N_2 = mvlmCluster.size();
    if (N_2 > 0){
        if( abs( mInfoSum/N_2 - infoTemp/N ) > mInfoSum/(N_2*3.0)){ 
            mvlmClusterCurrentFrame.clear();
            return false;
        }else{
            mvlmCluster.insert(mvlmCluster.end(), mvlmClusterCurrentFrame.begin(), mvlmClusterCurrentFrame.end())
            mvlmClusterCurrentFrame.clear();
            return true;
        }
    }

}

void Landmark::CalculateInfo(){

    size_t N = mvlmCluster.size();
    
    for(int i=0; i<N; i++){
        mInfoSum += mvlmCluster[i]->mEntropy;
    }

    mLastCluserSize = N;
}

void Landmark::UpdateInfo(){
    
    size_t N = mvlmCluster.size();

    for(int i=mLastCluserSize; i<N; i++){
        mInfoSum += mvlmCluster[i]->mEntropy;
    }

    mLastCluserSize = N;

}


} //namespace ORB_SLAM