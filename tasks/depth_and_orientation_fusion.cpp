/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "depth_and_orientation_fusion.hpp"

using namespace auv_helper;

depth_and_orientation_fusion::depth_and_orientation_fusion(std::string const& name)
    : depth_and_orientation_fusionBase(name)
{
}

depth_and_orientation_fusion::depth_and_orientation_fusion(std::string const& name, RTT::ExecutionEngine* engine)
    : depth_and_orientation_fusionBase(name, engine)
{
}
        
void depth_and_orientation_fusion::resetInitialHeading(){
    has_initial_heading=false;
}

depth_and_orientation_fusion::~depth_and_orientation_fusion()
{
}

void depth_and_orientation_fusion::depth_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    if(!depth_samples_sample.hasValidPosition(2)){
        error(GOT_INVALID_DEPTH);
        return;
    }
    
    lastDepth = depth_samples_sample;
    
    result.cov_velocity(2,2) = depth_samples_sample.cov_velocity(2,2);
    result.cov_position(2,2) = depth_samples_sample.cov_position(2,2);
    result.position[2] = depth_samples_sample.position[2] + actual_offset;
    result.velocity[2] = depth_samples_sample.velocity[2];
    result.time = depth_samples_sample.time;
}

void depth_and_orientation_fusion::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    if(!orientation_samples_sample.hasValidOrientation()){
        error(GOT_INVALID_ORIENTATION);
        return;
    }

    if(!has_initial_heading){
       _heading_offset.set(orientation_samples_sample.getYaw());
       has_initial_heading = true;
    }
    result.sourceFrame = orientation_samples_sample.sourceFrame;
    result.targetFrame = orientation_samples_sample.targetFrame;
    result.orientation = Eigen::AngleAxisd(-_heading_offset.get(),Eigen::Vector3d::UnitZ()) * orientation_samples_sample.orientation;
    result.cov_orientation = orientation_samples_sample.cov_orientation;
    result.angular_velocity = orientation_samples_sample.angular_velocity;
    result.cov_angular_velocity = orientation_samples_sample.cov_angular_velocity;
    result.time = orientation_samples_sample.time;
}

void depth_and_orientation_fusion::ground_distanceCallback(const base::Time &ts, const ::base::samples::RigidBodyState &ground_distance_sample){

  if(_depth_correction_buffer_size.get() > 0 && ground_distance_sample.hasValidPosition(2) &&
	ground_distance_sample.position[2] > 0.0 && lastDepth.hasValidPosition(2)){
    
    double sensor_depth = lastDepth.position(2);
    double echo_depth = _ground_depth.get() + ground_distance_sample.position(2);
    
    double diff = echo_depth - sensor_depth;
    
    offset_buffer.push_back(diff);
    
    actual_offset = calcMedian(); 
  }  
  
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See depth_and_orientation_fusion.hpp for more detailed
// documentation about them.

// bool depth_and_orientation_fusion::configureHook()
// {
//     if (! depth_and_orientation_fusionBase::configureHook())
//         return false;
//     return true;
// }
bool depth_and_orientation_fusion::startHook()
{
    if (! depth_and_orientation_fusionBase::startHook())
        return false;
    result.invalidate();
    lastDepth.invalidate();
    has_initial_heading = !_use_initial_heading.get();
    actual_offset = 0.0;
    
    offset_buffer.resize(_depth_correction_buffer_size.get());
    
    return true;
}
void depth_and_orientation_fusion::updateHook()
{
    depth_and_orientation_fusionBase::updateHook();

    if(!result.hasValidOrientation()){
        state(WAITING_FOR_ORIENTATION);
        return;
    }
    if(!result.hasValidPosition(2)){
        state(WAITING_FOR_DEPTH);
        return;
    }
    state(RUNNING);
    _pose_samples.write(result);

}

double depth_and_orientation_fusion::calcMedian(){
  
  if(!offset_buffer.empty() && _minimum_buffer_size.get() <= offset_buffer.size()){
   
    std::list<double> sorted_list;
    
    //Sort the buffered offsets
    for(boost::circular_buffer<double>::iterator it = offset_buffer.begin(); it != offset_buffer.end(); it++){
      
      if(sorted_list.empty()){
        sorted_list.push_back(*it);
      }else{
        
        std::list<double>::iterator jt = sorted_list.begin();
                
        while( jt != sorted_list.end() && *jt < *it)
          jt++;
        
        if(jt != sorted_list.end()){
          sorted_list.insert(jt, *it);
        }
        else{
          sorted_list.push_back(*it);
        }
        
      }     
    
    }

    //Return the middle element of the list
    std::list<double>::iterator jt = sorted_list.begin();
    
    //since there is no random access to lists, iterate to the middle element
    for(int i = 0; i < sorted_list.size()/2; i++)
      jt++;
    
    return *jt;    
  }
  
  return 0.0;  
}



// void depth_and_orientation_fusion::errorHook()
// {
//     depth_and_orientation_fusionBase::errorHook();
// }
// void depth_and_orientation_fusion::stopHook()
// {
//     depth_and_orientation_fusionBase::stopHook();
// }
// void depth_and_orientation_fusion::cleanupHook()
// {
//     depth_and_orientation_fusionBase::cleanupHook();
// }

