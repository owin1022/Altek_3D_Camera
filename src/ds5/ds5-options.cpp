// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.


#include "ds5/ds5-thermal-monitor.h"
#include "ds5-options.h"
#include "al3d-ai.h"

namespace librealsense
{
    const char* emitter_option::get_value_description(float val) const
    {
        switch (static_cast<int>(val))
        {
            case 0:
            {
                return "Off";
            }
            case 1:
            {
                return "Laser";
            }
            case 2:
            {
                return "Laser Auto";
            }
            case 3:
            {
                return "LED";
            }
            default:
                throw invalid_value_exception("value not found");
        }
    }

    emitter_option::emitter_option(uvc_sensor& ep)
        : uvc_xu_option(ep, ds::depth_xu, ds::DS5_DEPTH_EMITTER_ENABLED,
                        "Emitter select, 0-disable all emitters, 1-enable laser, 2-enable laser auto (opt), 3-enable LED (opt)")
    {}

    float asic_and_projector_temperature_options::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query is available during streaming only");

        #pragma pack(push, 1)
        struct temperature
        {
            uint8_t is_projector_valid;
            uint8_t is_asic_valid;
            int8_t projector_temperature;
            int8_t asic_temperature;
        };
        #pragma pack(pop)

        auto temperature_data = static_cast<temperature>(_ep.invoke_powered(
            [this](platform::uvc_device& dev)
            {
                temperature temp{};
                if (!dev.get_xu(ds::depth_xu,
                                ds::DS5_ASIC_AND_PROJECTOR_TEMPERATURES,
                                reinterpret_cast<uint8_t*>(&temp),
                                sizeof(temperature)))
                 {
                        throw invalid_value_exception(to_string() << "get_xu(ctrl=DS5_ASIC_AND_PROJECTOR_TEMPERATURES) failed!" << " Last Error: " << strerror(errno));
                 }

                return temp;
            }));

        int8_t temperature::* field;
        uint8_t temperature::* is_valid_field;

        switch (_option)
        {
        case RS2_OPTION_ASIC_TEMPERATURE:
            field = &temperature::asic_temperature;
            is_valid_field = &temperature::is_asic_valid;
            break;
        case RS2_OPTION_PROJECTOR_TEMPERATURE:
            field = &temperature::projector_temperature;
            is_valid_field = &temperature::is_projector_valid;
            break;
        default:
            throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }

        if (0 == temperature_data.*is_valid_field)
            LOG_ERROR(_ep.get_option_name(_option) << " value is not valid!");

        return temperature_data.*field;
    }

    option_range asic_and_projector_temperature_options::get_range() const
    {
        return option_range { -40, 125, 0, 0 };
    }

    bool asic_and_projector_temperature_options::is_enabled() const
    {
        return _ep.is_streaming();
    }

    const char* asic_and_projector_temperature_options::get_description() const
    {
        switch (_option)
        {
        case RS2_OPTION_ASIC_TEMPERATURE:
            return "Current Asic Temperature (degree celsius)";
        case RS2_OPTION_PROJECTOR_TEMPERATURE:
            return "Current Projector Temperature (degree celsius)";
        default:
            throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }
    }

    asic_and_projector_temperature_options::asic_and_projector_temperature_options(uvc_sensor& ep, rs2_option opt)
        : _option(opt), _ep(ep)
        {}

    float motion_module_temperature_option::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query is available during streaming only");

        static const auto report_field = platform::custom_sensor_report_field::value;
        auto data = _ep.get_custom_report_data(custom_sensor_name, report_name, report_field);
        if (data.empty())
            throw invalid_value_exception("query() motion_module_temperature_option failed! Empty buffer arrived.");

        auto data_str = std::string(reinterpret_cast<char const*>(data.data()));
        return std::stof(data_str);
    }

    option_range motion_module_temperature_option::get_range() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("get option range is available during streaming only");

        static const auto min_report_field = platform::custom_sensor_report_field::minimum;
        static const auto max_report_field = platform::custom_sensor_report_field::maximum;
        auto min_data = _ep.get_custom_report_data(custom_sensor_name, report_name, min_report_field);
        auto max_data = _ep.get_custom_report_data(custom_sensor_name, report_name, max_report_field);
        if (min_data.empty() || max_data.empty())
            throw invalid_value_exception("get_range() motion_module_temperature_option failed! Empty buffer arrived.");

        auto min_str = std::string(reinterpret_cast<char const*>(min_data.data()));
        auto max_str = std::string(reinterpret_cast<char const*>(max_data.data()));

        return option_range{std::stof(min_str),
                            std::stof(max_str),
                            0, 0};
    }

    bool motion_module_temperature_option::is_enabled() const
    {
        return _ep.is_streaming();
    }

    const char* motion_module_temperature_option::get_description() const
    {
        return "Current Motion-Module Temperature (degree celsius)";
    }

    motion_module_temperature_option::motion_module_temperature_option(hid_sensor& ep)
        : _ep(ep)
    {}

    void enable_motion_correction::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(enable_motion_correction) failed! Given value " << value << " is out of range.");

        _is_active = value > _opt_range.min;
        _recording_function(*this);
    }

    float enable_motion_correction::query() const
    {
        auto is_active = _is_active.load();
        return is_active ? _opt_range.max : _opt_range.min;
    }

    enable_motion_correction::enable_motion_correction(sensor_base* mm_ep,
                                                       const option_range& opt_range)
        : option_base(opt_range), _is_active(true)
    {}

    void enable_auto_exposure_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_exposure) failed! Invalid Auto-Exposure mode request " + std::to_string(value));

        auto auto_exposure_prev_state = _auto_exposure_state->get_enable_auto_exposure();
        _auto_exposure_state->set_enable_auto_exposure(0.f < std::fabs(value));

        if (_auto_exposure_state->get_enable_auto_exposure()) // auto_exposure current value
        {
            if (!auto_exposure_prev_state) // auto_exposure previous value
            {
                _to_add_frames = true; // auto_exposure moved from disable to enable
            }
        }
        else
        {
            if (auto_exposure_prev_state)
            {
                _to_add_frames = false; // auto_exposure moved from enable to disable
            }
        }
        _recording_function(*this);
    }

    float enable_auto_exposure_option::query() const
    {
        return _auto_exposure_state->get_enable_auto_exposure();
    }

    enable_auto_exposure_option::enable_auto_exposure_option(synthetic_sensor* fisheye_ep,
                                                             std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                             std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                             const option_range& opt_range)
        : option_base(opt_range),
          _auto_exposure_state(auto_exposure_state),
          _to_add_frames((_auto_exposure_state->get_enable_auto_exposure())),
          _auto_exposure(auto_exposure)
    {}

    auto_exposure_mode_option::auto_exposure_mode_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                         std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                         const option_range& opt_range,
                                                         const std::map<float, std::string>& description_per_value)
        : option_base(opt_range),
          _auto_exposure_state(auto_exposure_state),
          _auto_exposure(auto_exposure),
          _description_per_value(description_per_value)
    {}

    void auto_exposure_mode_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(auto_exposure_mode_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_mode(static_cast<auto_exposure_modes>((int)value));
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float auto_exposure_mode_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_mode());
    }

    const char* auto_exposure_mode_option::get_value_description(float val) const
    {
        try{
            return _description_per_value.at(val).c_str();
        }
        catch(std::out_of_range)
        {
            throw invalid_value_exception(to_string() << "auto_exposure_mode: get_value_description(...) failed! Description of value " << val << " is not found.");
        }
    }

    auto_exposure_step_option::auto_exposure_step_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
        std::shared_ptr<auto_exposure_state> auto_exposure_state,
        const option_range& opt_range)
        : option_base(opt_range),
        _auto_exposure_state(auto_exposure_state),
        _auto_exposure(auto_exposure)
    {}

    void auto_exposure_step_option::set(float value)
    {
        if (!std::isnormal(_opt_range.step) || ((value < _opt_range.min) || (value > _opt_range.max)))
            throw invalid_value_exception(to_string() << "set(auto_exposure_step_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_step(value);
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float auto_exposure_step_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_step());
    }

    auto_exposure_antiflicker_rate_option::auto_exposure_antiflicker_rate_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                                                 std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                                                 const option_range& opt_range,
                                                                                 const std::map<float, std::string>& description_per_value)
        : option_base(opt_range),
          _description_per_value(description_per_value),
          _auto_exposure_state(auto_exposure_state),
          _auto_exposure(auto_exposure)
    {}

    void auto_exposure_antiflicker_rate_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(auto_exposure_antiflicker_rate_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_antiflicker_rate(static_cast<uint32_t>(value));
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float auto_exposure_antiflicker_rate_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_antiflicker_rate());
    }

    const char* auto_exposure_antiflicker_rate_option::get_value_description(float val) const
    {
        try{
            return _description_per_value.at(val).c_str();
        }
        catch(std::out_of_range)
        {
            throw invalid_value_exception(to_string() << "antiflicker_rate: get_value_description(...) failed! Description of value " << val << " is not found.");
        }
    }

    ds::depth_table_control depth_scale_option::get_depth_table(ds::advanced_query_mode mode) const
    {
        command cmd(ds::GET_ADV);
        cmd.param1 = ds::etDepthTableControl;
        cmd.param2 = mode;
        auto res = _hwm.send(cmd);

        if (res.size() < sizeof(ds::depth_table_control))
            throw std::runtime_error("Not enough bytes returned from the firmware!");

        auto table = (const ds::depth_table_control*)res.data();
        return *table;
    }

    depth_scale_option::depth_scale_option(hw_monitor& hwm)
        : _hwm(hwm)
    {
        _range = [this]()
        {
            auto min = get_depth_table(ds::GET_MIN);
            auto max = get_depth_table(ds::GET_MAX);
            return option_range{ (float)(0.000001 * min.depth_units),
                                 (float)(0.000001 * max.depth_units),
                                 0.000001f, 0.001f };
        };
    }

    void depth_scale_option::set(float value)
    {
        command cmd(ds::SET_ADV);
        cmd.param1 = ds::etDepthTableControl;

        auto depth_table = get_depth_table(ds::GET_VAL);
        depth_table.depth_units = static_cast<uint32_t>(1000000 * value);
        auto ptr = (uint8_t*)(&depth_table);
        cmd.data = std::vector<uint8_t>(ptr, ptr + sizeof(ds::depth_table_control));

        _hwm.send(cmd);
        _record_action(*this);
        notify(value);
    }

    float depth_scale_option::query() const
    {
        auto table = get_depth_table(ds::GET_VAL);
        return (float)(0.000001 * (float)table.depth_units);
    }

    option_range depth_scale_option::get_range() const
    {
        return *_range;
    }

    external_sync_mode::external_sync_mode(hw_monitor& hwm, sensor_base* ep, int ver)
        : _hwm(hwm), _sensor(ep), _ver(ver)
    {
        _range = [this]()
        {
            return option_range{ ds::inter_cam_sync_mode::INTERCAM_SYNC_DEFAULT,
                                 static_cast<float>(_ver == 3 ? ds::inter_cam_sync_mode::INTERCAM_SYNC_MAX : (_ver == 2 ? 258 : 2)),
                                 1,
                                 ds::inter_cam_sync_mode::INTERCAM_SYNC_DEFAULT};
        };
    }

    void external_sync_mode::set(float value)
    {
        command cmd(ds::SET_CAM_SYNC);
        if (_ver == 1)
        {
            cmd.param1 = static_cast<int>(value);
        }
        else
        {
            if (_sensor->is_streaming())
                throw std::runtime_error("Cannot change Inter-camera HW synchronization mode while streaming!");

            if (value < 4)
                cmd.param1 = static_cast<int>(value);
            else if (value == 259) // For Sending two frame - First with laser ON, and the other with laser OFF.
            {
                cmd.param1 = 0x00010204; // genlock, two frames, on-off
            }
            else if (value == 260) // For Sending two frame - First with laser OFF, and the other with laser ON.
            {
                cmd.param1 = 0x00030204; // genlock, two frames, off-on
            }
            else
            {
                cmd.param1 = 4;
                cmd.param1 |= (static_cast<int>(value - 3)) << 8;
            }
        }

        _hwm.send(cmd);
        _record_action(*this);
    }

    float external_sync_mode::query() const
    {
        command cmd(ds::GET_CAM_SYNC);
        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("external_sync_mode::query result is empty!");

        if (res.front() < 4)
            return (res.front());
        else if (res[2] == 0x01)
        {
            return 259.0f;
        }
        else if (res[2] == 0x03)
        {
            return 260.0f;
        }
        else
            return (static_cast<float>(res[1]) + 3.0f);
    }

    option_range external_sync_mode::get_range() const
    {
        return *_range;
    }

    emitter_on_and_off_option::emitter_on_and_off_option(hw_monitor& hwm, sensor_base* ep)
        : _hwm(hwm), _sensor(ep)
    {
        _range = [this]()
        {
            return option_range{ 0, 1, 1, 0 };
        };
    }

    void emitter_on_and_off_option::set(float value)
    {
        if (_sensor->is_streaming())
            throw std::runtime_error("Cannot change Emitter On/Off option while streaming!");

        command cmd(ds::SET_PWM_ON_OFF);
        cmd.param1 = static_cast<int>(value);

        _hwm.send(cmd);
        _record_action(*this);
    }

    float emitter_on_and_off_option::query() const
    {
        command cmd(ds::GET_PWM_ON_OFF);
        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("emitter_on_and_off_option::query result is empty!");

        return (res.front());
    }

    option_range emitter_on_and_off_option::get_range() const
    {
        return *_range;
    }

    const char* external_sync_mode::get_description() const
    {
        if (_ver == 3)
            return "Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave, 3:Full Salve, 4-258:Genlock with burst count of 1-255 frames for each trigger, 259 and 260 for two frames per trigger with laser ON-OFF and OFF-ON.";
        else if (_ver == 2)
            return "Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave, 3:Full Salve, 4-258:Genlock with burst count of 1-255 frames for each trigger";
        else
            return "Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave";
    }

    alternating_emitter_option::alternating_emitter_option(hw_monitor& hwm, sensor_base* ep, bool is_fw_version_using_id)
        : _hwm(hwm), _sensor(ep), _is_fw_version_using_id(is_fw_version_using_id)
    {
        _range = [this]()
        {
            return option_range{ 0, 1, 1, 0 };
        };
    }

    void alternating_emitter_option::set(float value)
    {
        std::vector<uint8_t> pattern{};

        if (static_cast<int>(value))
        {
            if (_is_fw_version_using_id)
                pattern = ds::alternating_emitter_pattern;
            else
                pattern = ds::alternating_emitter_pattern_with_name;
        }

        command cmd(ds::SETSUBPRESET, static_cast<int>(pattern.size()));
        cmd.data = pattern;
        auto res = _hwm.send(cmd);
        _record_action(*this);
    }

    float alternating_emitter_option::query() const
    {
        if (_is_fw_version_using_id)
        {
            float rv = 0.f;
            command cmd(ds::GETSUBPRESETID);
            // if no subpreset is streaming, the firmware returns "ON_DATA_TO_RETURN" error
            try {
                auto res = _hwm.send(cmd);
                // if a subpreset is streaming, checking this is the alternating emitter sub preset
                if (res.size())
                    rv = (res[0] == ds::ALTERNATING_EMITTER_SUBPRESET_ID) ? 1.0f : 0.f;
            }
            catch (...)
            {
                rv = 0.f;
            }

            return rv;
        }
        else
        {
            command cmd(ds::GETSUBPRESETID);
            auto res = _hwm.send(cmd);
            if (res.size() > 20)
                throw invalid_value_exception("HWMON::GETSUBPRESETID invalid size");

            static std::vector<uint8_t> alt_emitter_name(ds::alternating_emitter_pattern_with_name.begin() + 2, ds::alternating_emitter_pattern_with_name.begin() + 22);
            return (alt_emitter_name == res);
        }
    }

    emitter_always_on_option::emitter_always_on_option(hw_monitor& hwm, sensor_base* ep)
        : _hwm(hwm), _sensor(ep)
    {
        _range = [this]()
        {
            return option_range{ 0, 1, 1, 0 };
        };
    }

    void emitter_always_on_option::set(float value)
    {
        command cmd(ds::LASERONCONST);
        cmd.param1 = static_cast<int>(value);

        _hwm.send(cmd);
        _record_action(*this);
    }

    float emitter_always_on_option::query() const
    {
        command cmd(ds::LASERONCONST);
        cmd.param1 = 2;

        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("emitter_always_on_option::query result is empty!");

        return (res.front());
    }

    option_range emitter_always_on_option::get_range() const
    {
        return *_range;
    }


    void hdr_option::set(float value)
    {
        _hdr_cfg->set(_option, value, _range);
        _record_action(*this);
    }

    float hdr_option::query() const
    {
        return _hdr_cfg->get(_option);
    }

    option_range hdr_option::get_range() const
    {
        return _range;
    }

    const char* hdr_option::get_value_description(float val) const
    {
        if (_description_per_value.find(val) != _description_per_value.end())
            return _description_per_value.at(val).c_str();
        return nullptr;
    }


    void hdr_conditional_option::set(float value)
    {
        if (_hdr_cfg->is_config_in_process())
            _hdr_option->set(value);
        else
        {
            if (_hdr_cfg->is_enabled())
                LOG_WARNING("The control - " << _uvc_option->get_description()
                     << " - is locked while HDR mode is active.\n"); 
            else
                _uvc_option->set(value);
        }
    }

    float hdr_conditional_option::query() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->query();
        else
            return _uvc_option->query();
    }

    option_range hdr_conditional_option::get_range() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->get_range();
        else
            return _uvc_option->get_range();
    }

    const char* hdr_conditional_option::get_description() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->get_description();
        else
            return _uvc_option->get_description();
    }

    bool hdr_conditional_option::is_enabled() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->is_enabled();
        else
            return _uvc_option->is_enabled();
    }

    auto_exposure_limit_option::auto_exposure_limit_option(hw_monitor& hwm, sensor_base* ep, option_range range)
        : option_base(range), _hwm(hwm), _sensor(ep)
    {
        _range = [range]()
        {
            return range;
        };
    }

    void auto_exposure_limit_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_exposure) failed! Invalid Auto-Exposure mode request " + std::to_string(value));

        command cmd_get(ds::AUTO_CALIB);
        cmd_get.param1 = 5;
        std::vector<uint8_t> ret = _hwm.send(cmd_get);
        if (ret.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        command cmd(ds::AUTO_CALIB);
        cmd.param1 = 4;
        cmd.param2 = static_cast<int>(value);
        cmd.param3 = *(reinterpret_cast<uint32_t*>(ret.data() + 4));
        _hwm.send(cmd);
        _record_action(*this);
    }

    float auto_exposure_limit_option::query() const
    {
        command cmd(ds::AUTO_CALIB);
        cmd.param1 = 5;

        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        return static_cast<float>(*(reinterpret_cast<uint32_t*>(res.data())));
    }

    option_range auto_exposure_limit_option::get_range() const
    {
        return *_range;
    }

    auto_gain_limit_option::auto_gain_limit_option(hw_monitor& hwm, sensor_base* ep, option_range range)
        : option_base(range), _hwm(hwm), _sensor(ep)
    {
        _range = [range]()
        {
            return range;
        };
    }

    void auto_gain_limit_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_gain) failed! Invalid Auto-Gain mode request " + std::to_string(value));

        command cmd_get(ds::AUTO_CALIB);
        cmd_get.param1 = 5;
        std::vector<uint8_t> ret = _hwm.send(cmd_get);
        if (ret.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        command cmd(ds::AUTO_CALIB);
        cmd.param1 = 4;
        cmd.param2 = *(reinterpret_cast<uint32_t*>(ret.data()));
        cmd.param3 = static_cast<int>(value);
        _hwm.send(cmd);
        _record_action(*this);
    }

    float auto_gain_limit_option::query() const
    {
        command cmd(ds::AUTO_CALIB);
        cmd.param1 = 5;

        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        return static_cast<float>(*(reinterpret_cast<uint32_t*>(res.data() + 4)));
    }

    option_range auto_gain_limit_option::get_range() const
    {
        return *_range;
    }

    librealsense::thermal_compensation::thermal_compensation(
        std::shared_ptr<ds5_thermal_monitor> monitor,
        std::shared_ptr<option> toggle) :
        _thermal_monitor(monitor),
        _thermal_toggle(toggle)
    {
    }

    float librealsense::thermal_compensation::query(void) const
    {
        auto val = _thermal_toggle->query();
        return val;
    }

    void librealsense::thermal_compensation::set(float value)
    {
        if (value < 0)
            throw invalid_value_exception("Invalid input for thermal compensation toggle: " + std::to_string(value));

        _thermal_toggle->set(value);
        _recording_function(*this);
    }

    const char* librealsense::thermal_compensation::get_description() const
    {
        return "Toggle thermal compensation adjustments mechanism";
    }

    const char* librealsense::thermal_compensation::get_value_description(float value) const
    {
        if (value == 0)
        {
            return "Thermal compensation is disabled";
        }
        else
        {
            return "Thermal compensation is enabled";
        }
    }

    //Work-around the control latency
    void librealsense::thermal_compensation::create_snapshot(std::shared_ptr<option>& snapshot) const
    {
        snapshot = std::make_shared<const_value_option>(get_description(), 0.f);
    }

    //al3d for fw_update
    al3d_fw_update::al3d_fw_update(uvc_sensor& ep)
        :  _ep(ep)
    {
  
    }
    bool al3d_fw_update::set_data_512(std::vector<uint8_t>& data) //512byte
    {
       
        auto rc = static_cast<bool>(_ep.invoke_powered(
            [this, &data](platform::uvc_device& dev)
            {
                size_t chunk_size = 512;
                if (data.size() < 512)
                    chunk_size = data.size();

                std::vector<uint8_t> transmit_buf(chunk_size, 0);  //512 or less
                std::copy(data.begin(), data.end(), transmit_buf.begin());

                if (!dev.set_xu(ds::fw_upgrade_xu,
                    ds::fw_upgrade_cs_set_data,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "set_xu failed!" << " Last Error: " << strerror(errno));
                }

                return 0;
            }));

        return 0;
    }
 
    bool al3d_fw_update::set_cmd(std::vector<uint8_t>& data) //8byte
    {
    
        auto rc = static_cast<bool>(_ep.invoke_powered(
            [this, &data ](platform::uvc_device& dev)
            {


                std::vector<uint8_t> transmit_buf(8, 0);  //8byte
                std::copy(data.begin(), data.end(), transmit_buf.begin());

                if (!dev.set_xu(ds::fw_upgrade_xu,
                    ds::fw_upgrade_cs_cmd,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                    ))
                {
                    throw invalid_value_exception(to_string() << "set_xu failed!" << " Last Error: " << strerror(errno));
                }

                return 0;
            }));

        return 0;
    }

    std::vector<uint8_t> al3d_fw_update::get_cmd() //8byte
    {

       return static_cast<std::vector<uint8_t>>(_ep.invoke_powered(
            [this](platform::uvc_device& dev)
            {


                std::vector<uint8_t> transmit_buf(8, 0);  //8byte
                if (!dev.get_xu(ds::fw_upgrade_xu,
                    ds::fw_upgrade_cs_result,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "get_xu failed!" << " Last Error: " << strerror(errno));
                }

                return transmit_buf;
            }));

    }

    //al3d for device commands
    al3d_device_xu_option::al3d_device_xu_option(uvc_sensor& ep)
        : _ep(ep)
    {

    }
    bool al3d_device_xu_option::set_PTS_Time(uint32_t host_second, uint32_t host_nanosecond) 
    {
        auto rc = static_cast<bool>(_ep.invoke_powered(
            [this, host_second, host_nanosecond](platform::uvc_device& dev)
            {

                std::vector<uint8_t> transmit_buf(8, 0);  //8byte
                uint8_t *p = transmit_buf.data();
                memcpy(p+0, (uint8_t*)&host_second, 4);
                memcpy(p+4, (uint8_t*)&host_nanosecond, 4);
                if (!dev.set_xu(ds::depth_xu,
                    ds::AL3D_Sync_PTS_Time,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "set_xu failed!" << " Last Error: " << strerror(errno));
                }
               

                return 0;
            }));

        return 0;
    }

    bool al3d_device_xu_option::get_PTS_Time(uint32_t* camera_second, uint32_t* camera_nanosecond)
    {
        auto rc = static_cast<bool>(_ep.invoke_powered(
            [this, camera_second, camera_nanosecond](platform::uvc_device& dev)
            {
                
                std::vector<uint8_t> transmit_buf(8, 0);  //8byte
                uint8_t* p = transmit_buf.data();
                if (!dev.get_xu(ds::depth_xu,
                    ds::AL3D_Sync_PTS_Time,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "get_xu failed!" << " Last Error: " << strerror(errno));
                }

                memcpy((uint8_t*)camera_second,p, 4);
                memcpy((uint8_t*)camera_nanosecond, p+4, 4);
                return 0;
            }));
        return 0;
    }

    bool al3d_device_xu_option::check_PTS_Time_Diff(uint32_t host_second, uint32_t host_nanosecond, uint32_t* diff_second, uint32_t* diff_nanosecond)
    {
        auto rc = static_cast<bool>(_ep.invoke_powered(
            [this, host_second, host_nanosecond, diff_second, diff_nanosecond](platform::uvc_device& dev)
            {

                std::vector<uint8_t> transmit_buf(8, 0);  //8byte
                uint8_t* p = transmit_buf.data();
                memcpy(p + 0, (uint8_t*)&host_second, 4);
                memcpy(p + 4, (uint8_t*)&host_nanosecond, 4);
                if (!dev.set_xu(ds::depth_xu,
                    ds::AL3D_PTS_Time_Diff,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "set_xu failed!" << " Last Error: " << strerror(errno));
                }

                if (!dev.get_xu(ds::depth_xu,
                    ds::AL3D_PTS_Time_Diff,
                    transmit_buf.data(),
                    static_cast<int>(transmit_buf.size())
                ))
                {
                    throw invalid_value_exception(to_string() << "get_xu failed!" << " Last Error: " << strerror(errno));
                }

                memcpy((uint8_t*)diff_second, p, 4);
                memcpy((uint8_t*)diff_nanosecond, p + 4, 4);

                return 0;
            }));

        return 0;
    }

	al3d_depth_cmd_option::al3d_depth_cmd_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, rs2_option opt, uint8_t read_opt, std::string name)
		: option_base(range), _hwm(hwm), _sensor(depth_ep), _opt_id(opt), _read_opt(read_opt)
	{
	    _name =  name + " -al3d";

        _range = [range]()
        {
            return range;
        };

		_value = (uint32_t)_range->def;
	}

	void al3d_depth_cmd_option::set(float value)
	{
		if (!is_valid(value))
			throw invalid_value_exception("set(al3d option) failed! " + std::to_string(value));

        if (is_read_only())
            return;

		_value = (int32_t) value;
		
		int p1 = _opt_id;
		int p2 =(_value & 0x00FF0000) >> 16;
		int p3 =(_value & 0x0000FF00) >> 8;
		int p4 =_value & 0x000000FF;

		command cmd(ds::fw_cmd::SET_AL3D_PARAM, p1, p2, p3, p4);
		_hwm.send(cmd);
		_record_action(*this);
	}

float al3d_depth_cmd_option::query() const
	{
		int32_t value;
		command cmd(ds::fw_cmd::SET_AL3D_PARAM, _opt_id,  0xff, 0x0, 0x0);
		std::vector<uint8_t> data;

		data = _hwm.send(cmd);
		
		if (data.empty())
			throw invalid_value_exception("al3d_depth_cmd_option::query result is empty!");
		
		memcpy((void*)&value, &data[8], sizeof(value));

		return static_cast<float>(value);
	}

	option_range al3d_depth_cmd_option::get_range() const
	{
		return *_range;
	}

	bool al3d_depth_cmd_option::is_read_only() const 
	{
		if(_read_opt == 1)
		{
			return true;
		}
		else if(_read_opt == 2)
		{
		 	return _sensor && _sensor->is_opened();
		}

		return false;
	}

	const char* al3d_depth_cmd_option::get_description() const 
	{
		return _name.data();
	}


   
    
    //------------------ for al3d_ai_cmd -------------------------
    al3d_ai_cmd_option::al3d_ai_cmd_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, rs2_option opt, uint8_t read_opt, std::string name)
        : option_base(range), 
          _hwm(hwm), 
          _sensor(depth_ep), 
          _opt_id(opt), 
          _read_opt(read_opt)
    {
        _name = name;

        _range = [range]()
        {
            return range;
        };

        _value = (uint32_t)_range->def;
    }


    void al3d_ai_cmd_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(al3d ai option) failed! " + std::to_string(value));

        if (is_read_only())
            return;

        int p1 = _opt_id;
        int p2 = al3d_ai_cmd_Set;
        int p3 = (uint32_t)value;
        int p4 = 0;

        command cmd(ds::fw_cmd::AL3D_AI_CMD, p1, p2, p3, p4);
        
        _hwm.send(cmd);
        _value = (uint32_t)value;

    }

    float al3d_ai_cmd_option::query() const
    {
        uint32_t value = 0;

        int p1 = _opt_id;
        int p2 = al3d_ai_cmd_Get;
        int p3 = 0;
        int p4 = 0;

        command cmd(ds::fw_cmd::AL3D_AI_CMD, p1, p2, p3, p4);
        std::vector<uint8_t> data;

        data = _hwm.send(cmd);

        //if (data.empty())
        //    throw invalid_value_exception("al3d_depth_cmd_option::query result is empty!");
        if (data.empty())
            return 0;


        memcpy((void*)&value, &data[0], sizeof(value));

        return static_cast<float>(value);
    }
        
    option_range al3d_ai_cmd_option::get_range() const
    {
        return *_range;
    }

    bool al3d_ai_cmd_option::is_read_only() const
    {
        if (_read_opt == 1)
        {
            return true;
        }
        else if (_read_opt == 2)
        {
            return _sensor && _sensor->is_opened();
        }

        return false;
    }

    const char* al3d_ai_cmd_option::get_description() const
    {
        return _name.data();
    }



    //------------------ end for al3d_ai_cmd -------------------------
   

}
