//License: Apache 2.0. See LICENSE file in root directory.
//Copyright(c) 2021 altek Corporation. All Rights Reserved.

#include "sensor.h"
#include <iostream>
#include <chrono>
#include "ds5-color.h"
#include "ds5-private.h"
#include "al3d-ai.h"

namespace librealsense
{
 //------------------ al3d ai al3d_ai_monitor ----------------

    al3d_ai_monitor::al3d_ai_monitor(
        std::shared_ptr<option> ai_option_enable,
        std::shared_ptr<option> ai_option_mode,
        hw_monitor& hwm) :
                _poll_intervals_ms(60),
                _thermal_threshold_deg(2.f),
                _temp_base(0.f),
                _hw_loop_on(false),
                _ai_option_enable(ai_option_enable),
                _ai_option_mode(ai_option_mode),
                _hwm(hwm)
    {
        _monitor = std::make_shared<active_object<>>([this](dispatcher::cancellable_timer cancellable_timer)
            {  polling(cancellable_timer);  });

        memset(&_ai_result_buffer[0], 0, 1016);
    }

    al3d_ai_monitor::~al3d_ai_monitor()
    {
        _monitor->stop();
        _temp_base = 0.f;
        _hw_loop_on = false;
    }

    void al3d_ai_monitor::update(bool on)
    {

        if (on) 
        {
            if (auto ai_enable_tmp = _ai_option_enable.lock())
            {
                auto cur_value = ai_enable_tmp->query();
               
                if (cur_value == 0)
                {
                    return;
                }

            }
            else
            {
                return;
            }
        }
      

        if (on != _monitor->is_active())
        {
            if (!on)
            {
                _monitor->stop();
                _hw_loop_on = false;
                // notify(0);
            }
            else
            {
                _hw_loop_on = true;
                _monitor->start();
            }
        }
    }

    void al3d_ai_monitor::polling(dispatcher::cancellable_timer cancellable_timer)
    {
		
		if(!_hw_loop_on)
			return;

        if (cancellable_timer.try_sleep(std::chrono::milliseconds(_poll_intervals_ms)))
        {
                command cmd(ds::fw_cmd::AL3D_AI_CMD, al3d_ai_cmd_AI_Result, al3d_ai_cmd_Get, 0x0, 0x0);
                std::vector<uint8_t> data;
             
                data = _hwm.send(cmd);

                if (data.empty()) 
                {
                    LOG_ERROR("Get AI Result fail: empty data");
                }
                else
                {
                    add_new_result((char*)data.data());
                }
				
        }
        else
        {
            LOG_DEBUG_THERMAL_LOOP("al3d_ai_monitor is being shut-down");
        }
    }

    void al3d_ai_monitor::notify(float temperature)
    {
#if 0
        _temp_base = temperature;
        for (auto&& cb : _thermal_changes_callbacks)
            cb(temperature);
#endif
    }

    void al3d_ai_monitor::add_new_result(char* new_result)
    {
       
   
        unsigned int m_u16TotalBytes, m_u16AI_BOX_Number = 0;
        memcpy(&m_u16TotalBytes, new_result + 0, 4);
        memcpy(&m_u16AI_BOX_Number, new_result + 4, 4);

        if ((m_u16TotalBytes > 0) && (m_u16TotalBytes < 1017) && (m_u16AI_BOX_Number > 0) && (m_u16AI_BOX_Number < 61))
        {
            result_buffer_lock.lock();
            memcpy(&_ai_result_buffer[0], new_result, 1016);
            result_buffer_lock.unlock();
        }

    }

    void al3d_ai_monitor::append_result(char* dst)
    {
        result_buffer_lock.lock();
        memcpy(dst ,&_ai_result_buffer[0], 1016);
        memset(&_ai_result_buffer[0], 0, 1016);
        result_buffer_lock.unlock();
    }
}
