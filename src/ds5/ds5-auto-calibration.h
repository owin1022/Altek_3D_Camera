// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "auto-calibrated-device.h"
#include "../core/advanced_mode.h"

namespace librealsense
{
    class auto_calibrated : public auto_calibrated_interface
    {
    public:
        auto_calibrated(std::shared_ptr<hw_monitor>& hwm);
        void write_calibration() const override;
        std::vector<uint8_t> run_on_chip_calibration(int timeout_ms, std::string json, float* health, update_progress_callback_ptr progress_callback) override;
        std::vector<uint8_t> run_tare_calibration(int timeout_ms, float ground_truth_mm, std::string json, update_progress_callback_ptr progress_callback) override;
        std::vector<uint8_t> get_calibration_table() const override;
        std::vector<uint8_t> get_calibration_table_by_id(int table_id) const override; //al3d: implement get al3d calibration data by id
        void set_calibration_table(const std::vector<uint8_t>& calibration) override;
        void reset_to_factory_calibration() const override;

    private:
        std::vector<uint8_t> get_calibration_results(float* health = nullptr) const;
        std::vector<uint8_t> get_PyRxFL_calibration_results(float* health = nullptr, float* health_fl = nullptr) const;
        void handle_calibration_error(int status) const;
        std::map<std::string, int> parse_json(std::string json);
        std::shared_ptr< ds5_advanced_mode_base> change_preset();
        void check_params(int speed, int scan_parameter, int data_sampling) const;
        void check_tare_params(int speed, int scan_parameter, int data_sampling, int average_step_count, int step_count, int accuracy);
        void check_focal_length_params(int step_count, int fy_scan_range, int keep_new_value_after_sucessful_scan, int interrrupt_data_samling, int adjust_both_sides, int fl_scan_location, int fy_scan_direction, int white_wall_mode) const;
        void check_one_button_params(int speed, int keep_new_value_after_sucessful_scan, int data_sampling, int adjust_both_sides, int fl_scan_location, int fy_scan_direction, int white_wall_mode) const;

        std::vector<uint8_t> _curr_calibration;
        std::shared_ptr<hw_monitor>& _hw_monitor;
    };

}
