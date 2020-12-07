/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>
#include<algorithm>
namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddLandmark(size_t l_id)
{
    unique_lock<mutex> lock(mMutexMap);
    Landmark* lM = new Landmark(l_id);
    mspLandmarks.insert(lM);
}

void Map::AddPointIntoLandmark(MapPoint* pMp)
{
    unique_lock<mutex> lock(mMutexMap);
    for(set<Landmark*>::iterator it=mspLandmarks.begin();it!=mspLandmarks.end();it++){
        if(pMp->mClassId == (*it)->mLandmarkClassId){
            (*it)->AddLandmarkPoint(pMp);
        }
    }
}

void Map::AddPointIntoCurrentLandmark(MapPoint* pMp)
{
    unique_lock<mutex> lock(mMutexMap);
    for(set<Landmark*>::iterator it=mspLandmarks.begin();it!=mspLandmarks.end();it++){
        if(pMp->mClassId == (*it)->mLandmarkClassId){
            (*it)->AddCurrentLandmarkPoint(pMp);
        }
    }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

void Map::IsGoodObservationInCurrentLandmark(){
    
    for(set<Landmark*>::iterator it=mspLandmarks.begin();it!=mspLandmarks.end();it++){
        if((*it)->mvlmClusterCurrentFrame.size() !=0){
            (*it)->isGoodObservation();
        }
    }
}

bool Map::IsMapPointInLandmark(MapPoint* pMp){

    for(set<Landmark*>::iterator it=mspLandmarks.begin();it!=mspLandmarks.end();it++){
        if(pMp->mClassId != (*it)->mLandmarkClassId){
            return false;
        }else{
            vector<MapPoint*>::iterator iter=find((*it)->mvlmCluster.begin(),(*it)->mvlmCluster.end(),pMp);
            if (iter == (*it)->mvlmCluster.end()){
                std::cout<<"Debug: no such point in any landmark" <<std::endl;
                return false;
            }else{
                std::cout<<"Debug: Find point any landmark" <<std::endl;
                return true;
            }
        }
    }
}

bool Map::IsNewLandmark(size_t l_id){
    for(set<Landmark*>::iterator it=mspLandmarks.begin();it!=mspLandmarks.end();it++){
        if(l_id== (*it)->mLandmarkClassId){
            return true;
        }        
    }
    return false;
}


vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::NumLandmarksInMap(){
    unique_lock<mutex> lock(mMutexMap);
    return mspLandmarks.size();
}


long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;
    for(set<Landmark*>::iterator sit=mspLandmarks.begin(), send=mspLandmarks.end(); sit!=send; sit++)
        delete *sit;
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspLandmarks.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
