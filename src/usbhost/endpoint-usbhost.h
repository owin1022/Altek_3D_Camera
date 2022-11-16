// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include <linux/usb/ch9.h>
#include "usb/usb-endpoint.h"

namespace librealsense
{
    namespace platform
    {
        class usb_endpoint_usbhost : public usb_endpoint
        {
        public:
            usb_endpoint_usbhost(usb_endpoint_descriptor desc, uint8_t interface_number) :
                    _desc(desc), _interface_number(interface_number)
                    { }

            usb_endpoint_usbhost(usb_endpoint_descriptor desc, uint8_t interface_number, usb_ss_ep_comp_descriptor ssdesc) :
                    _desc(desc), _interface_number(interface_number) , _ssdesc(ssdesc)
                    { }

            virtual uint8_t get_address() const override { return _desc.bEndpointAddress; }
            virtual uint16_t get_maxpacketsize() const override {
                if (_desc.bmAttributes == RS2_USB_ENDPOINT_ISOCHRONOUS) {
                    if (_ssdesc.wBytesPerInterval > 0 && _ssdesc.bMaxBurst > 0)
                        return _ssdesc.wBytesPerInterval;
                    else
                    return (_desc.wMaxPacketSize & 0x7FF) * ((_desc.wMaxPacketSize >> 11) + 1);
                }
                else
                    return _desc.wMaxPacketSize; }
            virtual endpoint_type get_type() const override { return (endpoint_type)_desc.bmAttributes; }
            virtual uint8_t get_interface_number() const override { return _interface_number; }

            virtual endpoint_direction get_direction() const override
            {
                return _desc.bEndpointAddress >= RS2_USB_ENDPOINT_DIRECTION_READ ?
                       RS2_USB_ENDPOINT_DIRECTION_READ : RS2_USB_ENDPOINT_DIRECTION_WRITE;
            }

            usb_endpoint_descriptor get_descriptor(){ return _desc; }

            usb_ss_ep_comp_descriptor get_ss_descriptor(){ return _ssdesc; }

        private:
            usb_endpoint_descriptor _desc;
            usb_ss_ep_comp_descriptor _ssdesc;
            uint8_t _interface_number;
        };
    }
}