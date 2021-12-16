#ifndef __TIME_BASED_RETRIEVER_H__
#define __TIME_BASED_RETRIEVER_H__

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <fstream>

static double time_diff_allowrance = 0.02;

template<typename T>
class time_based_retriever {
protected:
  typedef std::pair<ros::Time, T> Entry;

  T _data;
  std::vector<Entry> _data_list;
  bool _auto_delete;

public:
  time_based_retriever(bool auto_delete);
  void add_entry(ros::Time time, T entry);
  std::pair<ros::Time,T> get_closest_entry(const ros::Time &time, bool &success);
  void delete_previous(ros::Time time);
  std::vector< std::pair<ros::Time,T> > get_in_between(ros::Time &last_time, ros::Time &current_time, bool &success);
};

template<typename T>
time_based_retriever<T>::time_based_retriever(bool auto_delete): _auto_delete(auto_delete) {}

template<typename T>
void time_based_retriever<T>::add_entry(ros::Time time, T entry) 
{
  Entry e(time, entry);
  _data_list.push_back(e);
  ROS_DEBUG_STREAM(" Size of list:" << _data_list.size() << std::endl);
}

template<typename T>
std::pair<ros::Time,T> time_based_retriever<T>::get_closest_entry(const ros::Time &time, bool &success) 
{
  std::pair<ros::Time,T> entry;
  success = false;
  int delete_pos=0;

  if(_data_list.size() == 0) 
  {
    ROS_DEBUG_STREAM("No entries yet!" << std::endl);
    return entry;
  }
  
  ROS_DEBUG_STREAM("Trying to look up time: " << time << " first timestamp:" << _data_list[0].first << std::endl);

  //find the entry
  for(size_t i=0; i<_data_list.size(); i++) {

    if(time > _data_list[i].first) {
      //still in the past, get to the next entry
      ROS_DEBUG_STREAM("Continuing iteration :" << i << std::endl);
      continue;

    } else if(time == _data_list[i].first) {
      //found the exact entry
      entry = _data_list[i];
      delete_pos = i-1;
      success = true;
      ROS_DEBUG_STREAM("Found the exact entry" << std::endl);
      break;

    } else {
      if(i==0) {
        //time in the past
        success = false;
        ROS_DEBUG_STREAM("Time in past" << std::endl);
        return entry;
      }
      
      //check which time is closest
      if((time-_data_list[i-1].first) > (_data_list[i].first - time)) {
        entry = _data_list[i];
        delete_pos = i-1;
        success = true;
        ROS_DEBUG_STREAM("Found the entry at:" << i << std::endl);
        break;

      } else {
        entry = _data_list[i-1];
        delete_pos = i-2;
        success = true;
        ROS_DEBUG_STREAM("Found the i-1 entry:" << i-1 << std::endl);
        break;
      }
    } 
  }

  if(!success) {
    //return the last entry
    entry = _data_list[_data_list.size()-1];
    delete_pos = _data_list.size()-2;
    success = true;
  }

  ROS_DEBUG_STREAM("Delete position is:(inclusive) " << delete_pos << std::endl);
  //delete the entries prior to the current queried entry
  if(delete_pos > 0 && _auto_delete) {
    _data_list.erase(_data_list.begin(),_data_list.begin()+delete_pos+1);
    ROS_DEBUG_STREAM("Trimming the list to " << _data_list.size() << std::endl);
  }

  //return the entry found
  return entry;
}

template<typename T>
void time_based_retriever<T>::delete_previous(ros::Time time) {
  if(_auto_delete) {
    ROS_FATAL_STREAM("Auto delete set to true but delete called manually!!! Something wrong" << std::endl);
    exit(-1);
  }

  int delete_pos=0;
  bool success(false);

  if(_data_list.size() == 0) {
    ROS_DEBUG_STREAM("No entries yet!" << std::endl);
    return;
  }
  
  //find the entry
  for(size_t i=0; i<_data_list.size(); i++) {
    
    if(time > _data_list[i].first) {
      //still in the past, get to the next entry
      continue;

    } else if(time == _data_list[i].first) {
      //found the exact entry
      delete_pos = i-1;
      success = true;
      break;

    } else {
      if(i==0) {
        //time in the past
        success = false;
        return;
      }
      
      //check which time is closest
      if((time-_data_list[i-1].first) > (_data_list[i].first - time)) {
        delete_pos = i-1;
        success = true;
        break;

      } else {
        delete_pos = i-2;
        success = true;
        break;
      }
    } 
  }

  if(success) {
    ROS_DEBUG_STREAM("Delete position is: " << delete_pos << std::endl);
    //delete the entries prior to the current queried entry
    if(delete_pos > 0) {
      _data_list.erase(_data_list.begin(),_data_list.begin()+delete_pos);
    }
  }
}

template<typename T>
std::vector< std::pair<ros::Time,T> > time_based_retriever<T>::get_in_between(ros::Time &last_time, ros::Time &current_time, bool &success) {
  std::vector< std::pair<ros::Time,T> > entrys;
  success = false;
  int delete_pos=-1;

  if(_data_list.size() == 0) {
    ROS_DEBUG_STREAM("No entries yet!" << std::endl);
    return entrys;
  }
  if (last_time > current_time) {
    ROS_DEBUG_STREAM("last time is: " << last_time << " current_time is: " << current_time << std::endl);
    return entrys;
  }

  bool lastFound = false;
  size_t lastIndex = -1;
  //find the index of the last_time
  for(size_t i=0; i<_data_list.size(); i++) {
    // last_time is ahead of this time
    if(last_time > _data_list[i].first) {
      //still in the past, get to the next entry
      ROS_DEBUG_STREAM("Continuing iteration :" << i << std::endl);
      continue;
    }
    // last_time is exactly this time
    else if(last_time == _data_list[i].first) {
      //found the exact entry
      lastIndex = i;
      lastFound = true;
      //delete_pos = i-1;
      break;

    } 
    // last_time is before this time
    else {
      // before even the first
      if(i==0) {
        //last_time in the past too much
        if ((_data_list[0].first.toNSec()-last_time.toNSec())/1.0e9 > time_diff_allowrance) { // threshold on time diff
          ROS_DEBUG_STREAM("Time in past!! WRONG" << std::endl);
          return entrys;
        }
        // set as first index
        else {
          lastIndex = 0;
          lastFound = true;
          break;
        }
      }
      // befor the ith i>0
      else {
        //check which time is closest: ith is closer
        if((last_time-_data_list[i-1].first) > (_data_list[i].first - last_time)) {
          lastIndex = i;
          lastFound = true;
          //delete_pos = i-1;
          break;
        } 
        //check which time is closest: (i-1)th is closer
        else {
          lastIndex = i-1;
          lastFound = true;
          //delete_pos = i-2;
          break;
        }
      }
    }
  } 
  // can't find last_time, last time is larger than the most recent one
  if(!lastFound) {
    //last_time is ahead too much
    if ((last_time.toNSec()-_data_list[_data_list.size()-1].first.toNSec())/1.0e9 > time_diff_allowrance) {
      ROS_DEBUG_STREAM("Last time ahead too much. Wait for imu coming!" << std::endl);
      return entrys;
    } 
    // set as last index
    else {
      lastIndex = _data_list.size()-1;
      lastFound = true;
      //delete_pos = _data_list.size()-2;
    }
  }
  ROS_DEBUG_STREAM("Found the last entry at: " << lastIndex << std::endl);
  // for debug
  assert(lastFound==true);

  bool currFound = false;
  size_t currIndex = -1;
  // find the index of the current_time
  for (size_t i = lastIndex; i<_data_list.size(); i++) {
    // current_time is ahead of this time
    if(current_time > _data_list[i].first) {
      //still in the past, get to the next entry
      ROS_DEBUG_STREAM("Continuing iteration :" << i << std::endl);
      continue;
    }
    // current_time is exactly this time
    else if(current_time == _data_list[i].first) {
      //found the exact entry
      currIndex = i;
      currFound = true;
      delete_pos = i-1;
      break;
    } 
    // current_time is before this time
    else {
      // before even the first
      if(i==lastIndex) {
        // set as last index
        currIndex = lastIndex;
        currFound = true;
        delete_pos = lastIndex-1;
        ROS_DEBUG_STREAM("current_time and last_time map to the same imu index:" << lastIndex << std::endl);
        break;
      }
      // befor the ith i>lastIndex
      else {
        //check which time is closest: ith is closer
        if((current_time-_data_list[i-1].first) > (_data_list[i].first - current_time)) {
          currIndex = i;
          currFound = true;
          delete_pos = i-1;
          break;
        } 
        //check which time is closest: (i-1)th is closer
        else {
          currIndex = i-1;
          currFound = true;
          delete_pos = i-2;
          break;
        }
      }
    }
  }
  // can't find current, current time is larger than the most recent one
  if(!currFound) {
    //current_time is ahead too much
    if ((current_time.toNSec()-_data_list[_data_list.size()-1].first.toNSec())/1.0e9 > time_diff_allowrance) {
      ROS_DEBUG_STREAM("Current time ahead too much. Wait for imu coming!" << std::endl);
      return entrys;
    } 
    // set as current index
    else {
      currIndex = _data_list.size()-1;
      currFound = true;
      delete_pos = _data_list.size()-2;
    }
  }
  ROS_DEBUG_STREAM("Found the currnt entry at: " << currIndex << std::endl);

  // obtain result set success if both last and current succeed
  if (currFound && lastFound) {
    entrys = std::vector< std::pair<ros::Time,T> >(_data_list.begin()+lastIndex, _data_list.begin()+currIndex+1);
    success = true;
  }

  // delete if auto-delete
  ROS_DEBUG_STREAM("Delete position is:(inclusive) " << delete_pos << std::endl);
  // auto delete based on current_time
  if(delete_pos > 0 && _auto_delete) {
    _data_list.erase(_data_list.begin(),_data_list.begin()+delete_pos+1);
    ROS_DEBUG_STREAM("Trimming the list to " << _data_list.size() << std::endl);
  }
  return entrys;
}

#endif // __TIME_BASED_RETRIEVER_H__

