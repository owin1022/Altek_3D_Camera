// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#ifdef RS2_USE_ANDROID_BACKEND

#include <cstdint>
#include <vector>
#include "interface-usbhost.h"
#include "messenger-usbhost.h"

#include "../types.h"
#include "usbhost.h"

namespace librealsense
{
    namespace platform
    {
        usb_interface_usbhost::usb_interface_usbhost(::usb_interface_descriptor desc, ::usb_descriptor_iter it) :
                _desc(desc)
        {

  //          LOG_DEBUG("desc.bNumEndpoints = "  << desc.bNumEndpoints);
            for (int e = 0; e < desc.bNumEndpoints;) {
                usb_descriptor_header *h = usb_descriptor_iter_next(&it);
                if(h == NULL)
                    break;
                if (h->bDescriptorType == (USB_DT_ENDPOINT & ~USB_TYPE_MASK)) {
                    e++;
                    auto epd = *((usb_endpoint_descriptor *) h);
                    auto bkit = &it;
                    usb_descriptor_header *ssh = nullptr;
                    if (bkit != nullptr)
                        ssh = usb_descriptor_iter_next(bkit);
                    if (ssh != nullptr && ssh->bDescriptorType == USB_DT_SS_ENDPOINT_COMP)
                    {
                        auto ssepd = *((usb_ss_ep_comp_descriptor *) ssh);
                        auto ep = std::make_shared<usb_endpoint_usbhost>(epd,
                                                                         desc.bInterfaceNumber, ssepd);
                        _endpoints.push_back(ep);
                    }
                    else {
                        auto ep = std::make_shared<usb_endpoint_usbhost>(epd,
                                                                         desc.bInterfaceNumber);
                    _endpoints.push_back(ep);
                }
            }

            }
    //        LOG_DEBUG("endpoint size = "  << _endpoints.size());
        }

        usb_interface_usbhost::~usb_interface_usbhost()
        {

        }

        const rs_usb_endpoint usb_interface_usbhost::first_endpoint(endpoint_direction direction, endpoint_type type) const
        {
            for (auto&& ep : _endpoints)
            {
                if (ep->get_type() != type)
                    continue;
                if (ep->get_direction() != direction)
                    continue;
                return ep;
            }
            return nullptr;
        }

        void usb_interface_usbhost::add_associated_interface(const rs_usb_interface& interface)
        {
            if(interface)
                _associated_interfaces.push_back(interface);
        }
    }
}

#endif
