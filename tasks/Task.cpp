/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
#include <base/logging.h>
#include <act_schilling/Driver.hpp>
#include "Task.hpp"

using namespace act_schilling;

Task::Task(std::string const& name)
    : TaskBase(name),
      mDriver(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
      mDriver(0)
{
}

Task::~Task()
{
  delete mDriver;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

 bool Task::configureHook()
 {
   try{
      LOG_DEBUG("configureHook");
      delete mDriver;
      act_schilling::Config configuration = _config.get();
      mDriver = new act_schilling::Driver(configuration);
      if (!_io_port.get().empty())
      {
	  mDriver->open(_io_port.get());
      }

      setDriver(mDriver);
      if (! TaskBase::configureHook())
	  return false;
      return true;
    } catch(std::runtime_error &e){
      LOG_DEBUG("exception %s",e.what());
      error(COMM_ERROR);      
      return false;
    } 
}

bool Task::startHook()
 {
   try{
      LOG_DEBUG("start	Hook");
      
      if (! TaskBase::startHook())
	  return false;
      return true;
    } catch(std::runtime_error &e){
      LOG_DEBUG("exception %s",e.what());
      error(COMM_ERROR);      
      return false;
    } 
   
}

 void Task::updateHook()
 {
    try{
      //LOG_DEBUG("updateHook, state: %i",state());
      int val;
      if (_pos.read(val) == RTT::NewData) {
	LOG_DEBUG("input port pos changed");
        mDriver->setAnglePos(val);
      }
      if (_vel.read(val) == RTT::NewData) {
	LOG_DEBUG("input port pos changed");
        mDriver->setVelocity(val);
      }
      switch(state()){
	case RUNNING: {
	  mDriver->initDevice();
	  state(INIT_DEV);
	  break;
	}
	case INIT_DEV:{
	  if(mDriver->getState().initialized){
	    //mDriver->calibrate();
	    state(CAL_DEV);
	  }
	  break;
	}
	case CAL_DEV:{
	  if(mDriver->getState().calibrated){
	    state(RUN_DEV);
	  }
	}
	default: break;
      }      
      if(mDriver->isIdle()){
	mDriver->requestStatus();
      }
      mDriver->writeNext();
      _act_samples.write(mDriver->getData());
      TaskBase::updateHook();
    } catch(std::runtime_error &e){
      LOG_DEBUG("exception %s",e.what());
      error(COMM_ERROR);      
    }
 }
 
 void Task::errorHook()
 {
   LOG_DEBUG("errorHook");
   TaskBase::errorHook();
   recover();
 }

 void Task::stopHook()
 {
   TaskBase::stopHook();
 }

void Task::cleanupHook()
{
    if(mDriver){
      mDriver->close();
    }
    TaskBase::cleanupHook();
}

void Task::calibrate()
{
  LOG_DEBUG("calibrate");
  mDriver->calibrate();
}
