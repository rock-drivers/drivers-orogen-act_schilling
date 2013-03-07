/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
#include <base/logging.h>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <act_schilling/Driver.hpp>
#include <base_schilling/Error.hpp>
#include "Task.hpp"

#define MAX_VEL 2000

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
    //LOG_DEBUG("configureHook");
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
}

bool Task::startHook()
 {
    //LOG_DEBUG("startHook");
    RTT::extras::FileDescriptorActivity* fd_activity =
         getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity){
      fd_activity->watch(mDriver->getFileDescriptor());
      fd_activity->setTimeout(_io_read_timeout.get().toMilliseconds());
    }
    mDriver->setReadTimeout(_io_read_timeout.get());
    mDriver->setWriteTimeout(_io_write_timeout.get());
      
    if (! TaskBase::startHook())
      return false;
    return true;
}

 void Task::updateHook()
 {
   run();
 }
 
 void Task::errorHook()
 {
   LOG_DEBUG("errorHook");
   if(DEV_ERROR == state()){
     mDriver->clearError();
   }
   run();
   TaskBase::errorHook();
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
  try{
    mDriver->calibrate();
  } catch(std::runtime_error &e){
    LOG_DEBUG("exception %s",e.what());
    _log_message.write(LogMessage(e));
    exception(IO_TIMEOUT);
  }   
}

void Task::setControlMode(act_schilling::ControlMode  const & mode)
{
  try{
    mDriver->setControlMode(mode);
  } catch(std::runtime_error &e){
    LOG_DEBUG("exception %s",e.what());
    _log_message.write(LogMessage(e));
    exception(IO_TIMEOUT);
  }   
}

void Task::processIO()
{
  //LOG_DEBUG("processIO");
  mDriver->read();
  if(mDriver->hasStatusUpdate()){
    ActData data = mDriver->getData();
    _act_samples.write(data);
    ActDeviceStatus devStatus = mDriver->getDeviceStatus(); 
    _act_status.write(devStatus);
    statusCheck(devStatus);
  }
}

void Task::statusCheck(const ActDeviceStatus& devStatus)
{
  bool bDevError = false;
  if(devStatus.ctrl_status & ACT_CTRL_WD_TIME){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_WD_TIME, ACTALARM_CTRL_WD_TIME));
  }
  if(devStatus.ctrl_status & ACT_CTRL_SH_ENC_MAG){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_SH_ENC_MAG,ACTALARM_CTRL_SH_ENC_MAG));
  }
  if(devStatus.ctrl_status & ACT_CTRL_WATER){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_WATER,ACTALARM_CTRL_WATER));
  }
  if(devStatus.ctrl_status & ACT_CTRL_SH_ENC_COMM){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_SH_ENC_COMM,ACTALARM_CTRL_SH_ENC_COMM));
  }
  if(devStatus.drive_status & ACT_DRV_CMD_INC){
    bDevError = true;
    _log_message.write(LogMessage(Error, ACTSTR_DRV_CMD_INC,ACTALARM_DRV_CMD_INC));
  }
  if(devStatus.drive_status & ACT_DRV_CMD_INV){
    bDevError = true;
    _log_message.write(LogMessage(Error, ACTSTR_DRV_CMD_INV,ACTALARM_DRV_CMD_INV));
  }
  if(devStatus.drive_status & ACT_DRV_FRAME_ERR){
    bDevError = true;
    _log_message.write(LogMessage(Error, ACTSTR_DRV_FRAME_ERR,ACTALARM_DRV_FRAME_ERR));
  }
  if(devStatus.drive_status & ACT_DRV_VOLT_TEMP){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_DRV_VOLT_TEMP,ACTALARM_DRV_VOLT_TEMP));
  }
  if(devStatus.encoder_status & ACT_ENC_LIN_ALARM){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_ENC_LIN_ALARM,ACTALARM_ENC_LIN_ALARM));
  }
  if(devStatus.encoder_status & ACT_ENC_RANGE_ERR){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_ENC_RANGE_ERR,ACTALARM_ENC_RANGE_ERR));
  }
#ifdef ACT_EXT_ENC
  if(devStatus.ctrlStatus & ACT_CTRL_EXT_ENC_MAG){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_EXT_ENC_MAG,ACTALARM_CTRL_EXT_ENC_MAG));
  }
  if(devStatus.ctrlStatus & ACT_CTRL_EXT_ENC_COMM){
    bDevError = true;
    _log_message.write(LogMessage(Alarm, ACTSTR_CTRL_EXT_ENC_COMM,ACTALARM_CTRL_EXT_ENC_COMM));
  }
#endif
  if(bDevError){
    error(DEV_ERROR);
  }
  else{
    if(DEV_ERROR == state()){
      recover();
    }
  }
}


void Task::run()
{
   //LOG_DEBUG("run"); 
   try{
      switch(state()){
	case RUNNING: {
	  mDriver->clearReadBuffer();
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
	    state(MONITORING);
	  }	  
	  break;
	}
	case MONITORING:{
	  double d;
	  if (_doublevel.read(d) == RTT::NewData) {
	    mDriver->setVelocity(d*MAX_VEL);
	  }
	  int i;
	  if (_pos.read(i) == RTT::NewData) {
	    //LOG_DEBUG("input port pos changed");
	    mDriver->setAnglePos(i);
	  }
	  if (_vel.read(i) == RTT::NewData) {
	    //LOG_DEBUG("input port vel changed");
	    mDriver->setVelocity(i);
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
    } catch(std::runtime_error &e){
      LOG_DEBUG("exception %s",e.what());
      _log_message.write(LogMessage(e));
      exception(IO_TIMEOUT);
    }
}