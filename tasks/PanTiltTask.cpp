/* Generated from orogen/lib/orogen/templates/tasks/PanTiltTask.cpp */
#include <base/logging.h>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <act_schilling/Driver.hpp>
#include <base_schilling/Error.hpp>
#include "PanTiltTask.hpp"

#define MAX_VEL 2000

using namespace act_schilling;
using namespace oro_marum;

PanTiltTask::PanTiltTask(std::string const& name)
    : PanTiltTaskBase(name),
      mDriver(0)
{
}

PanTiltTask::PanTiltTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PanTiltTaskBase(name, engine),
      mDriver(0)
{
}

PanTiltTask::~PanTiltTask()
{
  delete mDriver;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PanTiltTask.hpp for more detailed
// documentation about them.

 bool PanTiltTask::configureHook()
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

    //    mJoystickMap = _joystickMap.get();
    //    mDefaultValButtonSize = mJoystickMap.defaultValButtons.size();
    //    mDefaultValButtonsOld = std::vector<uint8_t>(mDefaultValButtonSize,0);
    //    mVelocity=0;
    //    mSetDefaultPos =0;
    //    mSetDefaultFlag = false;
    //    mVelocityFlag = false;
    //    mTriggerButtonFlag = false;

    if (! PanTiltTaskBase::configureHook())
      return false;
    return true;
}

bool PanTiltTask::startHook()
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

    if (! PanTiltTaskBase::startHook())
      return false;
    return true;
}

 void PanTiltTask::updateHook()
 {
   run();
 }

 void PanTiltTask::errorHook()
 {
   LOG_DEBUG("errorHook");
   if(DEV_ERROR == state()){
     mDriver->clearError();
   }
   run();
   PanTiltTaskBase::errorHook();
 }

 void PanTiltTask::stopHook()
 {
   PanTiltTaskBase::stopHook();
 }

void PanTiltTask::cleanupHook()
{
    if(mDriver){
      mDriver->close();
    }
    PanTiltTaskBase::cleanupHook();
}

void PanTiltTask::calibrate()
{
  try{
    mDriver->calibrate();
  } catch(std::runtime_error &e){
    LOG_DEBUG("exception %s",e.what());
    _log_message.write(LogMessage(e));
    exception(IO_TIMEOUT);
  }
}

void PanTiltTask::setControlMode(act_schilling::ControlMode  const & mode)
{
  try{
    mDriver->setControlMode(mode);
  } catch(std::runtime_error &e){
    LOG_DEBUG("exception %s",e.what());
    _log_message.write(LogMessage(e));
    exception(IO_TIMEOUT);
  }
}

void PanTiltTask::processIO()
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

void PanTiltTask::statusCheck(const ActDeviceStatus& devStatus)
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


void PanTiltTask::run()
{
	//LOG_DEBUG("run");
	try{
		switch(state()){
			case RUNNING: {
				_act_calibstatus.write(true);
				mDriver->clearReadBuffer();
				mDriver->initDevice();
				state(INIT_DEV);
				break;
			}
			case INIT_DEV:{
				if(mDriver->getState().initialized){
					_act_calibstatus.write(false);
					state(CAL_DEV);
				}
				break;
			}
			case CAL_DEV:{
				if(mDriver->getState().calibrated){
					_act_calibstatus.write(true);
					_boundaries.set(mDriver->getBoundaries());
					state(MONITORING);
				}
				break;
			}
			case MONITORING:{
				int i;
				bool lock;
				double d;

				while (_rawCommand.read(mRawCmd) == RTT::NewData) {
					setRawCmd();
				}

				if(mControlMode != MODE_NONE){
					if (mVelocityFlag) {
						if(mControlMode == MODE_POS){
							mControlMode = MODE_VEL;
							mDriver->setControlMode(mControlMode);
						}
						mDriver->setVelocity(mVelocity*MAX_VEL);
						mVelocityFlag = false;
					}
					if (_pos.read(i) == RTT::NewData) {
						if(mControlMode == MODE_VEL){
							mControlMode = MODE_POS;
							mDriver->setControlMode(mControlMode);
						}
						LOG_DEBUG("input port pos changed");
						mDriver->setAnglePos(i);
					}
					if (mSetDefaultFlag) {
						if(mControlMode == MODE_VEL){
							mControlMode = MODE_POS;
							mDriver->setControlMode(mControlMode);
						}
						LOG_DEBUG("set default position");
						mDriver->setAnglePos(mSetDefaultPos);
						mSetDefaultFlag = false;
					}
				}
				if (_lock.read(lock) == RTT::NewData) {
					if(lock){
						mControlMode = MODE_NONE;
					} else {
						mControlMode = MODE_VEL;
					}
					mDriver->setControlMode(mControlMode);
				}
				break;
			}
			default: break;
		}

		mDriver->requestStatus();
		LOG_DEBUG("call WRITENEXT");

		mDriver->writeNext();
		LOG_DEBUG("WRITENEXT");

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

void PanTiltTask::setDefaultPos(int defPos){
  act_schilling::PanTiltDefaultPos defaultPos = _defaultPositions.get();
  mSetDefaultPos = defaultPos.pos_value[defPos];
  mSetDefaultFlag = true;
}

void PanTiltTask::setActualPosAsDefault(int defPos){
  act_schilling::ActData data = mDriver->getData();
  act_schilling::PanTiltDefaultPos defaultPos = _defaultPositions.get();
  defaultPos.pos_value[defPos] = (int)data.shaft_ang ;
  _defaultPositions.set(defaultPos);
}

void PanTiltTask::setRawCmd(){
   bool trigger = mRawCmd.buttonValue[mJoystickMap.triggerButton];


   if(trigger){
     //double max min +-2
     int val = mRawCmd.axisValue[mJoystickMap.inputAxisNumber][mJoystickMap.inputAxisDimension];
     double velocity = (double) val * 2 / 100;
     if(velocity != mVelocity){
       mVelocity = velocity;
       mVelocityFlag = true;
     }

     for(int i; i < mDefaultValButtonSize; i++) {
	 uint8_t value = mRawCmd.buttonValue[mJoystickMap.defaultValButtons[i]];

	 if(value != mDefaultValButtonsOld[i]){
	     if(value > 0){
		 setDefaultPos(i);
	     }
	     mDefaultValButtonsOld[i] = value;
	 }
     }

     if(mJoystickMap.configureButton > 0)
       if(mRawCmd.buttonValue[mJoystickMap.configureButton] > 0 && mDriver->getState().calibrated){
	   mDriver->setResetState();
	   state(RUNNING);
       }
     mTriggerButtonFlag = true;
   }

   if(mTriggerButtonFlag){
       mVelocity = 0;
       mVelocityFlag = true;

       mTriggerButtonFlag = false;
   }
 }
