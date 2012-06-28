/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
#include <base/logging.h>
#include <act_schilling/Driver.hpp>
#include <act_schilling/Error.hpp>
#include "Task.hpp"

using namespace act_schilling;
using namespace oro_marum;

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
      switch(state()){
	case RUNNING: {
	  mDriver->initDevice();
	  state(INIT_DEV);
	  break;
	}
	case INIT_DEV:{
	  if(mDriver->getState().initialized){
	    state(CAL_DEV);
	  }
	  break;
	}
	case CAL_DEV:{
	  if(mDriver->getState().calibrated){
	    _boundaries.set(mDriver->getBoundaries());
	    state(RUN_DEV);
	  }	  
	  break;
	}
	case RUN_DEV:{
	  double pos;
	  if (_pos.read(pos) == RTT::NewData) {
	    LOG_DEBUG("input port pos changed");
	    mDriver->setAnglePos(pos);
	  }
	  int vel;
	  if (_vel.read(vel) == RTT::NewData) {
	    LOG_DEBUG("input port vel changed");
	    mDriver->setVelocity(vel);
	  }
	  break;
	}
	default: break;
      }   
      mDriver->requestStatus();
      mDriver->writeNext();
      while(!mDriver->isIdle()){
	processIO();
	mDriver->writeNext();
      }
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
   mDriver->setResetState();
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
  mDriver->calibrate();
}

void Task::setControlMode(act_schilling::ControlMode  const & mode)
{
  mDriver->setControlMode(mode);
}

void Task::processIO()
{
  try{
    mDriver->read();
    if(mDriver->hasStatusUpdate()){
       ActData data = mDriver->getData();
       _act_samples.write(data);
       ActDeviceStatus devStatus = mDriver->getDeviceStatus(); 
       _act_status.write(devStatus);
       statusCheck(devStatus);
    }
  }catch(std::runtime_error& e){
    _log_message.write(LogMessage(e));
    error(COMM_ERROR);
  }
}

void Task::statusCheck(const ActDeviceStatus& devStatus)
{
  if(devStatus.ctrlStatus & ACT_CTRL_WD_TIME){
    _log_message.write(LogMessage(Alarm, ACTSTR_ENC_LIN_ALARM, ACTALARM_CTRL_WD_TIME));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_EXT_ENC_MAG){
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_EXT_ENC_MAG,ACTALARM_CTRL_EXT_ENC_MAG));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_EXT_ENC_COMM){
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_EXT_ENC_COMM,ACTALARM_CTRL_EXT_ENC_COMM));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_SH_ENC_MAG){
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_SH_ENC_MAG,ACTALARM_CTRL_SH_ENC_MAG));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_WATER){
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_WATER,ACTALARM_CTRL_WATER));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_SH_ENC_COMM){
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_SH_ENC_COMM,ACTALARM_CTRL_SH_ENC_COMM));
  }
  if(devStatus.driveStatus & ACT_DRV_CMD_INC){
    _log_message.write(LogMessage(Error, ACTSTR_DRV_CMD_INC,ACTALARM_DRV_CMD_INC));
  }
  if(devStatus.driveStatus & ACT_DRV_CMD_INV){
    _log_message.write(LogMessage(Error, ACTSTR_DRV_CMD_INV,ACTALARM_DRV_CMD_INV));
  }
  if(devStatus.driveStatus & ACT_DRV_FRAME_ERR){
    _log_message.write(LogMessage(Error, ACTSTR_DRV_FRAME_ERR,ACTALARM_DRV_FRAME_ERR));
  }
  if(devStatus.driveStatus & ACT_DRV_VOLT_TEMP){
    _log_message.write(LogMessage(Alarm, ACTSTR_DRV_VOLT_TEMP,ACTALARM_DRV_VOLT_TEMP));
  }
  if(devStatus.encoderStatus & ACT_ENC_LIN_ALARM){
    _log_message.write(LogMessage(Alarm, ACTSTR_ENC_LIN_ALARM,ACTALARM_ENC_LIN_ALARM));
  }
  if(devStatus.encoderStatus & ACT_ENC_RANGE_ERR){
    _log_message.write(LogMessage(Alarm, ACTSTR_ENC_RANGE_ERR,ACTALARM_ENC_RANGE_ERR));
  }
}