// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "ds5-private.h"

#include "algo.h"
#include "error-handling.h"
#include "../hdr-config.h"

namespace librealsense
{
    class emitter_option : public uvc_xu_option<uint8_t>
    {
    public:
        const char* get_value_description(float val) const override;
        explicit emitter_option(uvc_sensor& ep);
    };

    class asic_and_projector_temperature_options : public readonly_option
    {
    public:
        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override;

        const char* get_description() const override;

        explicit asic_and_projector_temperature_options(uvc_sensor& ep, rs2_option opt);

    private:
        uvc_sensor&                 _ep;
        rs2_option                  _option;
    };

    class motion_module_temperature_option : public readonly_option
    {
    public:
        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override;

        const char* get_description() const override;

        explicit motion_module_temperature_option(hid_sensor& ep);

    private:
        const std::string custom_sensor_name = "custom";
        const std::string report_name = "data-field-custom-value_2";
        hid_sensor& _ep;
    };

    class enable_auto_exposure_option : public option_base
    {
    public:
        void set(float value) override;

        float query() const override;

        bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Enable/disable auto-exposure";
        }

        auto_exposure_mechanism* get_auto_exposure() { return _auto_exposure.get(); }
        bool to_add_frames() { return _to_add_frames.load(); }

        enable_auto_exposure_option(synthetic_sensor* fisheye_ep,
                                    std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                    std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                    const option_range& opt_range);

    private:
        std::shared_ptr<auto_exposure_state>         _auto_exposure_state;
        std::atomic<bool>                            _to_add_frames;
        std::shared_ptr<auto_exposure_mechanism>     _auto_exposure;
    };

    class auto_exposure_mode_option : public option_base
    {
    public:
        auto_exposure_mode_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                  std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                  const option_range& opt_range,
                                  const std::map<float, std::string>& description_per_value);

        void set(float value) override;

        float query() const override;

        bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Auto-Exposure Mode";
        }

        const char* get_value_description(float val) const override;

    private:
        const std::map<float, std::string>          _description_per_value;
        std::shared_ptr<auto_exposure_state>        _auto_exposure_state;
        std::shared_ptr<auto_exposure_mechanism>    _auto_exposure;
    };

    class auto_exposure_step_option : public option_base
    {
    public:
        auto_exposure_step_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                  std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                  const option_range& opt_range);

        void set(float value) override;

        float query() const override;

        bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Auto-Exposure converge step";
        }

    private:
        std::shared_ptr<auto_exposure_state>        _auto_exposure_state;
        std::shared_ptr<auto_exposure_mechanism>    _auto_exposure;
    };

    class auto_exposure_antiflicker_rate_option : public option_base
    {
    public:
        auto_exposure_antiflicker_rate_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                              std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                              const option_range& opt_range,
                                              const std::map<float, std::string>& description_per_value);

        void set(float value) override;

        float query() const override;

        bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Auto-Exposure anti-flicker";
        }

        const char* get_value_description(float val) const override;

    private:
        const std::map<float, std::string>           _description_per_value;
        std::shared_ptr<auto_exposure_state>         _auto_exposure_state;
        std::shared_ptr<auto_exposure_mechanism>     _auto_exposure;
    };

    class depth_scale_option : public option, public observable_option
    {
    public:
        depth_scale_option(hw_monitor& hwm);
        virtual ~depth_scale_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Number of meters represented by a single depth unit";
        }
        void enable_recording(std::function<void(const option &)> record_action) override
        {
            _record_action = record_action;
        }

    private:
        ds::depth_table_control get_depth_table(ds::advanced_query_mode mode) const;
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
    };

    class external_sync_mode : public option
    {
    public:
        external_sync_mode(hw_monitor& hwm, sensor_base* depth_ep = nullptr, int ver = 1); // ver = 1, for firmware 5.9.15.1 and later, INTERCAM_SYNC_MAX is 2 with master and slave mode only.
                                                                                           // ver = 2, for firmware 5.12.4.0 and later, INTERCAM_SYNC_MAX is 258 by adding FULL SLAVE mode and genlock with trigger frequency 1 - 255.
                                                                                           // ver = 3  for firmware 5.12.12.100 and later, INTERCAM_SYNC_MAX is 260 by adding genlock with laser on-off and Off-On two frames.

        virtual ~external_sync_mode() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const override { return _sensor && _sensor->is_opened(); }
        const char* get_description() const override;

        void enable_recording(std::function<void(const option &)> record_action) override
        {
            _record_action = record_action;
        }
    private:
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
        int _ver;
    };

    class emitter_on_and_off_option : public option
    {
    public:
        emitter_on_and_off_option(hw_monitor& hwm, sensor_base* depth_ep);
        virtual ~emitter_on_and_off_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual const char* get_description() const override
        {
            return "Emitter On/Off Mode: 0:disabled(default), 1:enabled(emitter toggles between on and off). Can only be set before streaming";
        }
        virtual void enable_recording(std::function<void(const option &)> record_action) override {_record_action = record_action;}

    private:
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
    };

    class alternating_emitter_option : public option
    {
    public:
        alternating_emitter_option(hw_monitor& hwm, sensor_base* depth_ep, bool is_fw_version_using_id);
        virtual ~alternating_emitter_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override { return *_range; }
        virtual bool is_enabled() const override { return true; }
        virtual const char* get_description() const override
        {
            return "Alternating emitter pattern, toggled on/off on per-frame basis";
        }
        virtual void enable_recording(std::function<void(const option &)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
        bool _is_fw_version_using_id;
    };

    class emitter_always_on_option : public option
    {
    public:
        emitter_always_on_option(hw_monitor& hwm, sensor_base* depth_ep);
        virtual ~emitter_always_on_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual const char* get_description() const override
        {
            return "Emitter always on mode: 0:disabled(default), 1:enabled";
        }
        virtual void enable_recording(std::function<void(const option &)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
    };

    class hdr_option : public option
    {
    public:
        hdr_option(std::shared_ptr<hdr_config> hdr_cfg, rs2_option option, option_range range)
            : _hdr_cfg(hdr_cfg), _option(option), _range(range) {}

        hdr_option(std::shared_ptr<hdr_config> hdr_cfg, rs2_option option, option_range range, const std::map<float, std::string>& description_per_value)
            : _hdr_cfg(hdr_cfg), _option(option), _range(range), _description_per_value(description_per_value) {}

        virtual ~hdr_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual const char* get_description() const override { return "HDR Option"; }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }
        virtual const char* get_value_description(float) const override;

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        std::shared_ptr<hdr_config> _hdr_cfg;
        rs2_option _option;
        option_range _range;
        const std::map<float, std::string> _description_per_value;
    };

    // used for options that change their behavior when hdr configuration is in process
    class hdr_conditional_option : public option
    {
    public:
        hdr_conditional_option(std::shared_ptr<hdr_config> hdr_cfg,
            std::shared_ptr<option> uvc_option,
            std::shared_ptr<option> hdr_option):
            _hdr_cfg(hdr_cfg),
            _uvc_option(uvc_option),
            _hdr_option(hdr_option) {}

        virtual ~hdr_conditional_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override;
        virtual const char* get_description() const override;
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        std::shared_ptr<hdr_config> _hdr_cfg;
        std::shared_ptr<option> _uvc_option;
        std::shared_ptr<option> _hdr_option;
    };

    class auto_exposure_limit_option : public option_base
    {
    public:
        auto_exposure_limit_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range);
        virtual ~auto_exposure_limit_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const override { return _sensor && _sensor->is_opened(); }
        virtual const char* get_description() const override
        {
            return "Exposure limit is in microseconds. Default is 0 which means full exposure range. If the requested exposure limit is greater than frame time, it will be set to frame time at runtime. Setting will not take effect until next streaming session.";
        }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
    };

    class auto_gain_limit_option : public option_base
    {
    public:
        auto_gain_limit_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range);
        virtual ~auto_gain_limit_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const override { return _sensor && _sensor->is_opened(); }
        virtual const char* get_description() const override
        {
            return "Gain limits ranges from 16 to 248. Default is 0 which means full gain. If the requested gain limit is less than 16, it will be set to 16. If the requested gain limit is greater than 248, it will be set to 248. Setting will not take effect until next streaming session.";
        }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
    };

    class al3d_depth_cmd_option : public option_base
    {
    public:
        al3d_depth_cmd_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, rs2_option opt, uint8_t read_opt, std::string name);
        virtual ~al3d_depth_cmd_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const override;
        virtual const char* get_description() const override;
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }
    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
		rs2_option     _opt_id;
		int32_t _value;
		std::string _name;
		uint8_t _read_opt;
    };

    class ds5_thermal_monitor;
    class thermal_compensation : public option
    {
    public:
        thermal_compensation(std::shared_ptr<ds5_thermal_monitor> monitor,
            std::shared_ptr<option> toggle);

        void set(float value) override;
        float query() const override;

        option_range get_range() const override { return option_range{ 0, 1, 1, 0 }; }
        bool is_enabled() const override { return true; }

        const char* get_description() const override;
        const char* get_value_description(float value) const override;
        void create_snapshot(std::shared_ptr<option>& snapshot) const override;

        void enable_recording(std::function<void(const option&)> record_action) override
        {
            _recording_function = record_action;
        }

    private:
        std::shared_ptr<ds5_thermal_monitor>  _thermal_monitor;
        std::shared_ptr<option> _thermal_toggle;

        std::function<void(const option&)> _recording_function = [](const option&) {};
    };


    //al3d fw_update
    class al3d_fw_update 
    {
    public:
        al3d_fw_update( uvc_sensor& ep);
        ~al3d_fw_update() = default;
        bool set_data_512(std::vector<uint8_t>& data);
        bool set_cmd(std::vector<uint8_t>& data);
        std::vector<uint8_t> get_cmd();
    private:
        uvc_sensor& _ep;
   
    };
    //al3d_device_xu_option
    class al3d_device_xu_option
    {
    public:
        al3d_device_xu_option(uvc_sensor& ep);
        ~al3d_device_xu_option() = default;
       
        bool set_PTS_Time(uint32_t host_second, uint32_t host_nanosecond);
        bool get_PTS_Time(uint32_t *camera_second, uint32_t *camera_nanosecond);
        bool check_PTS_Time_Diff(uint32_t host_second, uint32_t host_nanosecond, uint32_t* diff_second, uint32_t* diff_nanosecond);
       
    private:
        uvc_sensor& _ep;

    };
   

    
    class al3d_ai_cmd_option : public option_base
    {
    public:
        al3d_ai_cmd_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, rs2_option opt, uint8_t read_opt, std::string name);
        virtual ~al3d_ai_cmd_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const override;
        virtual const char* get_description() const override;
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }
    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
        rs2_option     _opt_id;
        uint32_t _value;
        std::string _name;
        uint8_t _read_opt;
       
    };



}
