// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "request-usbhost.h"
#include "endpoint-usbhost.h"
#include "device-usbhost.h"

//#define USE_LIBLOG

#ifdef USE_LIBLOG
#define LOG_TAG "uvcstream"
#include "android/log.h"
//#define DD ALOGD
#define DD(...) __android_log_print(ANDROID_LOG_DEBUG, "uvcstream", __VA_ARGS__)
#else
#define DD(...) {}
#endif

namespace librealsense
{
    namespace platform
    {
        usb_request_usbhost::usb_request_usbhost(rs_usb_device device, rs_usb_endpoint endpoint )
        {
            _endpoint = endpoint;
            auto dev = std::static_pointer_cast<usb_device_usbhost>(device);
            auto read_ep = std::static_pointer_cast<usb_endpoint_usbhost>(_endpoint);
            auto desc = read_ep->get_descriptor();
            auto ssdesc = read_ep->get_ss_descriptor();
            /*
            if ((desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC) {
                LOG_DEBUG("endpoint addr =" << std::hex << (uint16_t) desc.bEndpointAddress << std::dec
                                            << " type = " << (uint16_t) _endpoint->get_type() << std::dec
                                            << " dir = " << (uint16_t) _endpoint->get_direction() << "max packget ="
                                            << std::dec << (uint16_t) desc.wMaxPacketSize);
            } */
            _native_request = std::shared_ptr<::usb_request>(usb_request_new(dev->get_handle(), &desc , &ssdesc),
             [this](::usb_request* req)
             {
                 if(!_active)
                     usb_request_free(req);
                 else
                     LOG_ERROR("active request didn't return on time");
             });
            _native_request->client_data = this;
        }

        usb_request_usbhost::~usb_request_usbhost()
        {
            DD("----- ~usb_request_usbhost _active = %d  @%p" , (int)_active , _native_request.get()->private_data);
            if(_active)
                usb_request_cancel(_native_request.get());

            int attempts = 10;
            while(_active && attempts--)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        void usb_request_usbhost::set_active(bool state)
        {
            _active = state;
        }

        int usb_request_usbhost::get_native_buffer_length()
        {
            return _native_request->buffer_length;
        }

        void usb_request_usbhost::set_native_buffer_length(int length)
        {
            _native_request->buffer_length = length;
        }

        int usb_request_usbhost::get_actual_length() const
        {
            return _native_request->actual_length;
        }

        void usb_request_usbhost::set_native_buffer(uint8_t* buffer)
        {
            _native_request->buffer = buffer;
        }

        uint8_t* usb_request_usbhost::get_native_buffer() const
        {
            return (uint8_t*)_native_request->buffer;
        }

        void* usb_request_usbhost::get_native_request() const
        {
            return _native_request.get();
        }

        std::shared_ptr<usb_request> usb_request_usbhost::get_shared() const
        {
            return _shared.lock();
        }

        void usb_request_usbhost::set_shared(const std::shared_ptr<usb_request>& shared)
        {
            _shared = shared;
        }
    }
}
