//License: Apache 2.0. See LICENSE file in root directory.
//Copyright(c) 2021 altek Corporation. All Rights Reserved.

#pragma once
#include "sensor.h"


namespace librealsense
{
    struct _ALTEK_AI_BOX_INFO_
    {
        unsigned short m_u16Box_ID;
        unsigned short m_u16Box_Left;
        unsigned short m_u16Box_Top;
        unsigned short m_u16Box_Right;
        unsigned short m_u16Box_Bottom;
        unsigned short m_u16Box_Distance;
        float m_f32Box_Degree;
    };

    enum al3d_ai_cmd
    {
        al3d_ai_cmd_base = 0,
        al3d_ai_cmd_AI_Result,
        al3d_ai_cmd_AI_Enable,
        al3d_ai_cmd_AI_Mode,
        al3d_ai_cmd_max_id,
    };

    enum al3d_ai_cmd_get_set
    {
        al3d_ai_cmd_Get = 0,
        al3d_ai_cmd_Set,
    };


   class  al3d_ai_monitor
    {
    public:
        al3d_ai_monitor(
            std::shared_ptr<option> ai_option_enable,
            std::shared_ptr<option> ai_option_mode,
            hw_monitor& hwm );
        ~al3d_ai_monitor();

        void update(bool on);
        void add_observer(std::function<void(float)> callback)
        {
            _thermal_changes_callbacks.push_back(callback);
        }
        void set_polling_interval_ms(unsigned int intervals_ms)
        {
            _poll_intervals_ms = intervals_ms;
        }
        void add_new_result(char* new_result);
        void append_result(char* dst);


    private:
        al3d_ai_monitor(const al3d_ai_monitor&) = delete;       // disable copy and assignment ctors
        al3d_ai_monitor& operator=(const al3d_ai_monitor&) = delete;

        // Active Object's main routine
        void polling(dispatcher::cancellable_timer cancellable_timer);
        void notify(float  temperature);
        std::shared_ptr < active_object<> > _monitor;
        unsigned int _poll_intervals_ms;
        float _thermal_threshold_deg;
        float _temp_base;
        bool _hw_loop_on;
        std::weak_ptr<option> _ai_option_enable;
        std::weak_ptr<option> _ai_option_mode;
        std::vector<std::function<void(float)>>  _thermal_changes_callbacks;   // Distribute notifications on device thermal changes
        hw_monitor& _hwm;
        std::mutex result_buffer_lock;
        char _ai_result_buffer[1016];
    };

}
