// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <usbhost/usbhost.h>
#include <linux/usbdevice_fs.h>
#include "uvc-streamer.h"
#include "../backend.h"
//#include "utilbase.h"

//#define USE_LIBLOG
#ifdef USE_LIBLOG
#undef LOG_TAG
#define LOG_TAG "uvcstream"
#include "android/log.h"
//#define DD ALOGD
#define DD(...) __android_log_print(ANDROID_LOG_DEBUG, "uvcstream", __VA_ARGS__)
#else
#define DD(...) {}
#endif

const int UVC_PAYLOAD_MAX_HEADER_LENGTH         = 1024;
const int DEQUEUE_MILLISECONDS_TIMEOUT          = 50;
const int ENDPOINT_RESET_MILLISECONDS_TIMEOUT   = 100;
const int MAX_ISO_BUFFER_LENGTH	 =	32768;	// 32 x 1024 = 32KB
const int MAX_REQUEST_COUNT	 =	2;

void cleanup_frame(backend_frame *ptr) {
    if (ptr) {
        int orgsize = ptr->owner->get_size();
        if (orgsize > 0)
            ptr->owner->deallocate(ptr);
        LOG_DEBUG(" ----cleanup_frame ---@" << ptr <<" " /*<< (int) ptr->pixels.size() << "  " */<< orgsize << "  " <<ptr->owner->get_size() ); //<< " owner=" << std::hex << ptr->owner << " ptr=" << ptr
    }
}

namespace librealsense
{
    namespace platform
    {
        uvc_streamer::uvc_streamer(uvc_streamer_context context) :
            _context(context), _action_dispatcher(10)
        {
            LOG_DEBUG("uvc_streamer bInterfaceNumber: " << (uint32_t) context.control->bInterfaceNumber );
            auto inf = context.usb_device->get_interface_isoc(context.control->bInterfaceNumber , context.control->dwMaxPayloadTransferSize );
            if (inf == nullptr)
                inf = context.usb_device->get_interface(context.control->bInterfaceNumber ); // bulk mode
            if (inf == nullptr)
                throw std::runtime_error("can't find UVC streaming interface of device: " + context.usb_device->get_info().id);

            _read_endpoint = inf->first_endpoint(platform::RS2_USB_ENDPOINT_DIRECTION_READ );
            if (_read_endpoint == nullptr)
                _read_endpoint = inf->first_endpoint(platform::RS2_USB_ENDPOINT_DIRECTION_READ , RS2_USB_ENDPOINT_ISOCHRONOUS);
            if (_read_endpoint == nullptr)
                throw std::runtime_error("can't find endpoint for streaming :  bulk and isoc" );

            LOG_DEBUG("interface  = " << (uint32_t) inf->get_number() <<  " Alt = " <<(uint32_t) inf->get_alternate_setting() );
            //set interface start stream
            if (_read_endpoint->get_type() == RS2_USB_ENDPOINT_ISOCHRONOUS) {
                _context.messenger->set_interface(inf->get_number(), inf->get_alternate_setting());
                //calc num of iso desc
            //    _context.request_count = (_context.control->dwMaxVideoFrameSize+_context.control->dwMaxPayloadTransferSize-1)  / _context.control->dwMaxPayloadTransferSize;
            //    _context.request_count =  _context.request_count  /  (MAX_ISO_BUFFER_LENGTH / _context.control->dwMaxPayloadTransferSize);
                _context.request_count = MAX_REQUEST_COUNT;
                if (_context.request_count > MAX_REQUEST_COUNT) _context.request_count = MAX_REQUEST_COUNT;
                    _read_buff_length = UVC_PAYLOAD_MAX_HEADER_LENGTH + _context.control->dwMaxVideoFrameSize;
                LOG_DEBUG ( "context.request_count  = "  <<  (uint32_t) _context.request_count  );
             //     if (_read_buff_length > 32*3072) {
             //        _read_buff_length = 32 * 3072;
             //       LOG_DEBUG("---- linit iso mode _read_buff_length 32k -----");
             //  }
            }
            else {
                _context.request_count = 2;// test
                _read_buff_length =
                        UVC_PAYLOAD_MAX_HEADER_LENGTH + _context.control->dwMaxVideoFrameSize;
            }
            //LOG_DEBUG("endpoint -+++++++++" );
       //    LOG_DEBUG("endpoint 0x"<< std::hex  << (int)_read_endpoint->get_address() << " read buffer size: " << std::dec <<_read_buff_length);

            _action_dispatcher.start();

            _watchdog_timeout = (1000.0 / _context.profile.fps) * 100;

            LOG_DEBUG("endpoint 0x"<< std::hex  << (int)_read_endpoint->get_address() << " read buffer size: " << std::dec <<_read_buff_length << " tiemout " <<  _watchdog_timeout);

            //  _context.request_count = 10;
            init();
        }

        uvc_streamer::~uvc_streamer()
        {
            flush();
        }

        void uvc_process_bulk_payload(backend_frame_ptr fp, size_t payload_len, backend_frames_queue& queue) {

            /* ignore empty payload transfers */
            if (!fp || payload_len < 2)
                return;

            uint8_t header_len = fp->pixels[0];
            uint8_t header_info = fp->pixels[1];

            size_t data_len = payload_len - header_len;

            if (header_info & 0x40)
            {
                LOG_ERROR("bad packet: error bit set");
                return;
            }
            if (header_len > payload_len)
            {
                LOG_ERROR("bogus packet: actual_len=" << payload_len << ", header_len=" << header_len);
                return;
            }


      //      LOG_DEBUG("Passing packet to user CB with size " << (data_len + header_len) << " data_len " << data_len << " header_len " << (int)header_len);
            librealsense::platform::frame_object fo{ data_len, header_len,
                                                     fp->pixels.data()+ header_len , fp->pixels.data() };//
            fp->fo = fo;

            queue.enqueue(std::move(fp));
        }

        void uvc_streamer::init()
        {
            _urb_process_count =0;
            _gframe_count = 0;
            _gfid = 0;
            _ggot_bytes = 0;
            _gptr = nullptr;
            _greusedptr = nullptr;
          //  _queueadded = true;
            _frames_archive = std::make_shared<backend_frames_archive>();
            // Get all pointers from archive and initialize their content
            std::vector<backend_frame *> frames;
            for (auto i = 0; i < _frames_archive->CAPACITY; i++) {
                auto ptr = _frames_archive->allocate();
                ptr->pixels.resize(_read_buff_length, 0);
                ptr->owner = _frames_archive.get();
                frames.push_back(ptr);
            }

            for (auto ptr : frames) {
                _frames_archive->deallocate(ptr);
            }

            _publish_frame_thread = std::make_shared<active_object<>>([this](dispatcher::cancellable_timer cancellable_timer)
            {
                backend_frame_ptr fp(nullptr, [](backend_frame *) {});
                if (_queue.dequeue(&fp, DEQUEUE_MILLISECONDS_TIMEOUT))
                {
                    if(_publish_frames && running())
                        _context.user_cb(_context.profile, fp->fo, []() mutable {});
                }
            });

            _watchdog = std::make_shared<watchdog>([this]()
             {
                 _action_dispatcher.invoke([this](dispatcher::cancellable_timer c)
                   {
                       if(!_running || !_frame_arrived)
                           return;

                       LOG_ERROR("uvc streamer watchdog triggered on endpoint:0x " << std::hex << (int)_read_endpoint->get_address() << " " << (int)_frame_arrived);
                  //     _context.messenger->reset_endpoint(_read_endpoint, ENDPOINT_RESET_MILLISECONDS_TIMEOUT); //can not reset randy test
                       _frame_arrived = false;
                   });
             }, _watchdog_timeout);

            _watchdog->start();

            _request_callback = std::make_shared<usb_request_callback>([this](platform::rs_usb_request r)
            {
                _action_dispatcher.invoke([this, r](dispatcher::cancellable_timer)
                {
                    if(!_running)
                      return;
                    auto type = r->get_endpoint()->get_type();
                    auto interface_num = r->get_endpoint()->get_interface_number();
                    auto al = r->get_actual_length();
                    // Relax the frame size constrain for compressed streams
                    bool is_compressed = val_in_range(_context.profile.format, {0x4d4a5047U,
                                                                                0x5a313648U}); // MJPEG, Z16H

               //     if (interface_num == 1 ) return;  //disable depth
                    if (type == RS2_USB_ENDPOINT_BULK) {

                        if (al > 0L && ((al == r->get_buffer().data()[0] +
                                               _context.control->dwMaxVideoFrameSize) ||
                                        is_compressed)) {
                            auto f = backend_frame_ptr(_frames_archive->allocate(), &cleanup_frame);

                            if (f) {
                                _frame_arrived = true;

                                memcpy(f->pixels.data(), r->get_buffer().data(),
                                       r->get_buffer().size());
                                LOG_DEBUG("uvc_process_bulk_payload " << r->get_buffer().size()
                                                                      << "get_actual_length "
                                                                      << r->get_actual_length()
                                                                      << " pixels.size "
                                                                      << f->pixels.size());
                                uvc_process_bulk_payload(std::move(f), r->get_actual_length(),
                                                         _queue);

                            }
                        }
                    }
                    else if (type == RS2_USB_ENDPOINT_ISOCHRONOUS) {
                        /* per packet */
                        uint8_t *pktbuf;
                        uint8_t check_header;
                        size_t header_len;
                        uint8_t header_info;
                        int packet_id;
                        struct usbdevfs_iso_packet_desc *pkt;
                    //    static uint8_t gfid = 0;
                       // static uint8_t grgbfid = 0;
                      //  static uint8_t gdepthfid = 0;
                  //      static uint32_t ggot_bytes = 0;
                    //    uint32_t gbkgot_bytes = 0;
                     //   static uint32_t grgbgot_bytes =0;
                     //   static uint32_t gdepthgot_bytes =0;
                     //   static backend_frame *gptr = nullptr;
                    //    static backend_frame *gdepthptr = nullptr;
                    //    static backend_frame *grgbptr = nullptr;
                      //  static backend_frame *greusedptr = nullptr;
                  //      static backend_frame *greused_rgbptr = nullptr;
                  //      static backend_frame *greused_depthptr = nullptr;

#if 0
// enable for adjust setting the UI
  if (interface_num > 0 ) //randy temp solution
                        {
  LOG_DEBUG("interface  " <<  (int)interface_num << " depth sensor ");
                            return ;
                        }
#endif
                        if (al >0) {
                            _urb_process_count++;
                           // std::lock_guard<std::mutex> lock(_iso_mutex);
#if 0
                            if (interface_num == 1) //randy temp solution
                            {
                                gptr = gdepthptr;
                                greusedptr = greused_depthptr;
                               // gfid = gdepthfid;
                              //  ggot_bytes = gdepthgot_bytes;
                                if (gptr != nullptr && (gptr == greused_rgbptr || gptr == grgbptr))
                                    LOG_ERROR("--- error -- depth use color buffer ");
                            } else {
                                gptr = grgbptr;
                              //  gfid = grgbfid;
                                greusedptr = greused_rgbptr;
                             //   ggot_bytes = grgbgot_bytes;
                                if (gptr != nullptr && (gptr == gdepthptr || gptr == greused_depthptr))
                                    LOG_ERROR("--- error -- color use depth buffer ");
                            }
#endif
                       //     gbkgot_bytes = _ggot_bytes;

                            auto nr = reinterpret_cast<::usb_request *>(r->get_native_request());
                            struct usbdevfs_urb *urb = (struct usbdevfs_urb *) nr->private_data;

#if 0
                            if (al > 0L)
                                LOG_DEBUG("al = " << std::dec << (uint32_t) al
                                                  << " " << _urb_process_count
                                                  //<< "@" << std::hex << urb
                                                  //<< " get_buffer = "
                                                  //<< (uint32_t) r->get_buffer().data()[0]
                                                  //<< " 0x" << std::hex
                                                  //<< (uint32_t) r->get_buffer().data()[1]
                                                  //   << _context.control->dwMaxVideoFrameSize
                                                  // << " get_buffer().size()  "
                                                  // << r->get_buffer().size()
                                                  //  << " get_actual_length "
                                                  //  << r->get_actual_length()
                                                  << " inf " << (int) interface_num
                                                  << " size " << (int) _frames_archive->get_size()
                                                  << " queue  " << (int) _queue.size()
                                                  << std::hex
                                                  << " gptr=@" << _gptr
                                                  //<< " gdepthptr@" << gdepthptr
                                                  //<< " grgbptr@" << grgbptr
                                                  << " greusedptr@" << _greusedptr
                                                  << std::dec
                                                 // << " bkgot_bytes " << (int) gbkgot_bytes
                                                  << " ggot_bytes " << (int) _ggot_bytes
                                                  << " gframe_count " << (int) _gframe_count
                                // << " " << gdepthgot_bytes <<  " " << grgbgot_bytes
                                );
#endif
                           // if (_frames_archive->get_size() - _queue.size() > 3)
                            //    LOG_ERROR("--- Large unused frame------ ");
                            //   DD("[ urb @%p status = %d, actual = %d number_of_packets %d ]\n", urb, urb->status, urb->actual_length , urb->number_of_packets);
                            if (urb->actual_length > 0 && urb->buffer) {
                                uint8_t *pbuf = (uint8_t *) urb->buffer;
                                //   DD("actual_length=%d 0x%02x, 0x%02x " ,  urb->actual_length , pbuf[0] , pbuf[1]);
                            }
                            if (urb == nullptr) return;
                            if (urb->buffer == nullptr) return;
                            //  if (urb) LOG_DEBUG("type " <<(uint16_t) urb->type << " num_iso_packets" <<(uint16_t) urb->number_of_packets);

                            if (urb->actual_length > 0)
                                for (packet_id = 0;
                                     packet_id < urb->number_of_packets; ++packet_id) {
                                    check_header = 1;
                                    pkt = urb->iso_frame_desc + packet_id;
                                    //     LOG_DEBUG("id " <<(uint16_t) packet_id << " urb:status=0x " << std::hex << (uint16_t)pkt->status << " actual_length= " <<(uint16_t) pkt->actual_length);
                                    if ((pkt->status != 0)) {
                                        LOG_WARNING("bad packet:status= " << std::hex << pkt->status
                                                                          << " actual_length= "
                                                                          << pkt->actual_length);
                                        //    strmh->bfh_err |= UVC_STREAM_ERR;
                                        continue;
                                    }
                                    if (0 ==
                                        pkt->actual_length) {    // why transfered byte is zero...
                                        //  LOG_DEBUG("zero packet (transfer):");
                                        continue;
                                    }
                                    pktbuf = (uint8_t *) urb->buffer +
                                             ((int) urb->iso_frame_desc[0].length * packet_id);
                                    if ((pktbuf)) {
                                        header_len = pktbuf[0];    // Header length field of Stream Header

                                        if ((check_header)) {
                                            header_info = pktbuf[1];
                                            if (0 != (header_info & UVC_STREAM_ERR)) {
                                                //		strmh->bfh_err |= UVC_STREAM_ERR;
                                                LOG_WARNING("UVC_STREAM_ERR bad packet:status= 0x"
                                                                    << std::hex
                                                                    << (int) header_info
                                                                    << std::dec
                                                                    << " packet_id "<< packet_id
                                                                    << " inf= " << (int)interface_num
                                                                    << "gotbyte= " << _ggot_bytes
                                                                    << std::hex << "_gptr=@" << _gptr
                                                                    << std::dec << " " << _urb_process_count);
                                                //      libusb_clear_halt(strmh->devh->usb_devh, strmh->stream_if->bEndpointAddress);

                                                if (_gptr) _greusedptr = _gptr;
                                                _ggot_bytes = 0;
                                                //continue;
                                                break;
                                            }

                                            if ((_gfid != (header_info & UVC_STREAM_FID)) &&
                                                _ggot_bytes >0) {
                                                LOG_DEBUG("FID change ggot_bytes =" << _ggot_bytes << " inf = "
                                                                                   << (int) interface_num << " "
                                                                                   << " "
                                                                                  /*
                                                                                   // << " owner get size = " << gptr->owner->get_size()

                                                                                   << " "
                                                                                   << std::hex
                                                                                   << (uint16_t) (
                                                                                           header_info &
                                                                                           UVC_STREAM_FID)
                                                                                   << " "
                                                                                   << std::dec  */
                                                                                   << " packet_id = "<< packet_id
                                                                                   << " size = "<< (int) _frames_archive->get_size()
                                                                                   << std::hex << " _gptr=@" << _gptr
                                                                                    << std::dec << " " << _urb_process_count );
                                                /*if (gptr)
                                                {
                                                    if ( _frames_archive->get_size() > 0 && ggot_bytes != _context.control->dwMaxVideoFrameSize) {
                                                        greusedptr = gptr;
                                                        DD("FID change reused gptr = @%p = " , greusedptr);
                                                    }
                                                } */
                                                /*if (gptr)
                                                    if ( _frames_archive->get_size() > 0) {
                                                        gptr->owner->deallocate(gptr);
                                                        DD("FID free _frames_archive size = %d inf = %d",
                                                           _frames_archive->get_size() , interface_num);
                                                        ggot_bytes = 0;
                                                    } */
                                                _gfid = header_info & UVC_STREAM_FID;
                                                if (_ggot_bytes != _context.control->dwMaxVideoFrameSize) {
                                                    _ggot_bytes = 0;
                                                    _greusedptr = _gptr;
                                                    continue;
                                                }

                                            }
                                            _gfid = header_info & UVC_STREAM_FID;

                                            if (header_info & UVC_STREAM_PTS) {
                                                // XXX saki some camera may send broken packet or failed to receive all data
                                                // LOG_ERROR("bad packet stream pts");
                                                //    continue;
                                            }

                                            if (header_info & UVC_STREAM_SCR) {
                                                // XXX saki some camera may send broken packet or failed to receive all data
                                                //LOG_ERROR("bad packet stream scr");
                                                //    continue;
                                            }

                                        } // if LIKELY(check_header)

                                        if ((pkt->actual_length < header_len)) {
                                            /* Bogus packet received */
                                            //    strmh->bfh_err |= UVC_STREAM_ERR;
                                            LOG_WARNING(
                                                    "bad packet: actual_len " << pkt->actual_length
                                                                              << "header_len "
                                                                              << header_len);
                                            continue;
                                        }

                                        if ((pkt->actual_length > header_len)) {
                                            const size_t odd_bytes =
                                                    pkt->actual_length - header_len;
                                            // assert(ggot_bytes + odd_bytes < strmh->size_buf);
                                            // assert(strmh->outbuf);
                                            assert(pktbuf);
                                            if (_ggot_bytes == 0) {
                                                if (_frames_archive->get_size() < 0) {
                                                    LOG_WARNING("----error frame archive < 0 ---"
                                                                        << " packet_id = "
                                                                        << packet_id);
                                                }
                                                //    LOG_DEBUG(" _frames_archive =" << std::hex <<_frames_archive);
                                              //  if (greusedptr != nullptr && gptr != greusedptr)
                                               //     DD("---------error ----greusedptr= @%p gptr = @%p ",
                                                //       greusedptr, gptr);
                                                if (_gptr == _greusedptr && _gptr != nullptr &&
                                                    _frames_archive->get_size() > 0) {
                                                    //DD("reused _frames_archive size = %d inf = %d  gptr=%p",
                                                    //   _frames_archive->get_size(), interface_num,
                                                    //   gptr);
                                                    LOG_DEBUG(" reused _frames_archive inf="
                                                                      << (int) interface_num
                                                                      << " size = "
                                                                      << (int) _frames_archive->get_size()
                                                                      << std::hex <<
                                                                      " _frames_archive = "
                                                                      << _frames_archive
                                                                      << " gptr= " << _gptr);
                                                    _greusedptr = nullptr;
                                                } else {
                                                   // if (_queueadded|| _frames_archive->get_size() == 0 || _queue.size() == 0) //
                                                    _gptr = _frames_archive->allocate();

                                                    // DD("alloced _frames_archive size = %d inf = %d  gptr=%p", ,
                                                    //     interface_num,_frames_archive->get_size(),
                                                    //    gptr);
                                                    if (_gptr)
                                                        LOG_DEBUG(" alloced _frames_archive inf="
                                                                      << (int) interface_num
                                                                      << " size = "
                                                                      << _frames_archive->get_size()
                                                                      << std::hex
                                                                     // " _frames_archive = "
                                                                     // << _frames_archive
                                                                      << " gptr= " << _gptr
                                                                      << std::dec
                                                                      << " packet_id = "<< packet_id
                                                                      );
                                                    else
                                                        LOG_ERROR(" fail alloc buffer full ------"
                                                                          << (int) interface_num
                                                                          << " size = "
                                                                          << _frames_archive->get_size()
                                                                          << std::hex
                                                                          // " _frames_archive = "
                                                                          // << _frames_archive
                                                                          << " gptr= " << _gptr
                                                                          << std::dec
                                                                          << " packet_id = "<< packet_id
                                                                         // << " _queueadded " << _queueadded
                                                                          << " queue " << _queue.size()
                                                                          << " framecount= " << _gframe_count
                                                        );
                                                 //   _queueadded = false;
                                                }

                                               /* if (_frames_archive->get_size() == 6) {
                                                    LOG_WARNING("----frame size bigger ---"
                                                                        << " packet_id = "
                                                                        << packet_id);
                                                }*/

                                                //backend_frame_ptr f =  backend_frame_ptr(_frames_archive->allocate(), nullptr);//&cleanup_frame
                                                if (_gptr) {
                                                    _frame_arrived = true;
                                                    memcpy(_gptr->pixels.data() + _ggot_bytes, pktbuf,
                                                           pkt->actual_length);
                                                }
                                            } else if (_gptr) {
                                                //         memcpy(strmh->outbuf + ggot_bytes, pktbuf+header_len, odd_bytes);
                                                //     if (ggot_bytes+odd_bytes <= )
                                                if (_gptr->pixels.size() >= _ggot_bytes + odd_bytes)
                                                    memcpy(_gptr->pixels.data() + _ggot_bytes,
                                                           pktbuf + header_len,
                                                           odd_bytes);
                                            }
                                            _ggot_bytes += odd_bytes;
                                            /*LOG_DEBUG("ggot = " << ggot_bytes
                                                             << " odd = " << odd_bytes
                                                             << " actual = " << pkt->actual_length
                                                             << " head = " << header_len
                                                             << " inf_num = " << (int)interface_num); */
                                        }

                                        if (((pktbuf[1] & UVC_STREAM_EOF) || (_ggot_bytes ==
                                                                              _context.control->dwMaxVideoFrameSize)) &&
                                            _ggot_bytes != 0) {
                                            /* The EOF bit is set, so publish the complete frame */
                                            if ((!is_compressed) &&
                                                (_ggot_bytes !=
                                                 _context.control->dwMaxVideoFrameSize)) {
                                                LOG_ERROR(
                                                        "Error: EOF frame bytes  = " << _ggot_bytes
                                                                                   << "  "
                                                                                   << _context.control->dwMaxVideoFrameSize
                                                                                   << " "<< (int) interface_num
                                                                                 //  << " " << gbkgot_bytes
                                                                                   << " packet_id = "<< packet_id
                                                                                   //<< " number_of_packets " << urb->number_of_packets
                                                                                   << " " << _gframe_count
                                                                                   << " " << _urb_process_count);
                                                if (_gptr) {
                                                    _greusedptr = _gptr;
                                                    //DD("reused gptr = @%p = ", _greusedptr);
                                                    // (gptr)->owner->deallocate(gptr);
                                                    // DD("EOF error free _frames_archive size = %d inf = %d",
                                                    //    _frames_archive->get_size() , interface_num);
                                                }
                                                //     gptr = nullptr;
                                                _ggot_bytes = 0;
                                               // break;
                                                continue;
                                            } else {


                                                //   LOG_DEBUG("EOF frame bytes = " << ggot_bytes);
                                                if (_gptr != nullptr) {
                                                    DD("EOF frame bytes = %d inf = %d  size = %d _queue  =%d packet_id=%d",
                                                       _ggot_bytes,
                                                       interface_num, _frames_archive->get_size(),
                                                       _queue.size(), packet_id);
                                                }
                                                else {
                                                    LOG_ERROR("---------------gptr is NULL");
                                                    continue;
                                                }
                                                // if (gframe_count > 1) {  //randy test
                                                if ( //_queue.size() <= 2 ||
                                                    /*_frames_archive->get_size() <= 10 */ 1) {
                                                    backend_frame *bkptr = _gptr;
                                                    // DD("@gptr=@%p" , gptr);
                                                    auto f = backend_frame_ptr(_gptr,
                                                                               &cleanup_frame);
                                                    if (f) {
                                                        if (bkptr != _gptr) {
                                                            LOG_ERROR("Error:---abnormal error");
                                                            break;
                                                        }
                                                        if (_ggot_bytes !=
                                                            _context.control->dwMaxVideoFrameSize) {
                                                            LOG_ERROR("Error:---unknow error ----"
                                                                              << " ggot_bytes = "
                                                                              << _ggot_bytes
                                                                              << " dwMaxVideoFrameSize=  "
                                                                              << _context.control->dwMaxVideoFrameSize
                                                                             );
                                                            break;
                                                        }
                                                        _gframe_count++;
                                                        _frame_arrived = true;
                                                    //    DD("@gptr=@%p ggot_bytes = %d gptr->pixels.size() = %d packet_id = %d",
                                                    //       _gptr, _ggot_bytes, _gptr->pixels.size(),
                                                    //       packet_id);
                                                     //   _queueadded = true;
                                                        uvc_process_bulk_payload(std::move(f),
                                                                                 _ggot_bytes +
                                                                                 header_len,
                                                                                 _queue);
                                                        _greusedptr = nullptr;
                                                    }
                                                } else {
                                                    //   if (gptr) _frames_archive->deallocate(gptr);
                                                    if (_gptr) {
                                                        _greusedptr = _gptr;
                                                        LOG_ERROR("----queue full -- inf = " <<  (int)interface_num );
                                                     //   DD("queue full reused gptr = @%p = inf=%d ",
                                                     //      _greusedptr, interface_num);
                                                    }
                                                    // if (_frames_archive->get_size()> 0) {
                                                    //     gptr->owner->deallocate(gptr);
                                                    //     DD("queue full free _frames_archive size = %d inf = %d",
                                                    //        _frames_archive->get_size() , interface_num);
                                                    // }
                                                    LOG_ERROR("Error:----backend_frame_ptr null");
                                                }
                                         //       DD("_after gptr=@%p frame size = %d _queue size = %d inf = %d ggot_bytes = %d packet_id=%d _gframe_count =%d", _gptr,
                                         //         _frames_archive->get_size(), (int)_queue.size(),
                                         //          interface_num, _ggot_bytes , packet_id , _gframe_count);

                                                //     if (gbkptr) gbkptr->owner->deallocate(gbkptr);
                                                //    DD("delloc frames  size remain = %d \n" , _frames_archive->get_size());
                                                // }
                                                //    gptr = nullptr;
                                                //      gbkptr = gptr;
                                                _ggot_bytes = 0;
                                                //continue;
                                                break;
                                            }
                                        } else {    // if (LIKELY(pktbuf))
                                            //  strmh->bfh_err |= UVC_STREAM_ERR;
                                            //       LOG_DEBUG("pktbuf returned null");
                                            continue;
                                        }
                                    }
                                }
                   //         if (greused_depthptr == greused_rgbptr && greused_depthptr != nullptr)
                   //             LOG_ERROR("----error----used ptr the same---");
#if 0
                            if (interface_num == 1) //randy temp solution
                            {
                                gdepthptr = _gptr;
                             //   gdepthfid = gfid;
                                //gdepthgot_bytes = ggot_bytes;
                                greused_depthptr = _greusedptr;
                                if (greused_depthptr != nullptr)
                                    DD("---greused_depthptr = @%p", greused_depthptr);
                                //     DD("end depth gotbyte = %d" , gdepthgot_bytes );
                            } else {
                                grgbptr = _gptr;
                             //   grgbgot_bytes = ggot_bytes;
                             //   grgbfid = gfid;
                                greused_rgbptr = _greusedptr;
                                if (greused_rgbptr != nullptr)
                                    DD("---greused_rgbptr = @%p", greused_rgbptr);
                                //    DD("end rgb gotbyte = %d" , grgbgot_bytes );
                            }
#endif
                        }
                    }
                        auto sts = _context.messenger->submit_request(r);
                        if (sts != platform::RS2_USB_STATUS_SUCCESS)
                            LOG_ERROR("failed to submit UVC request, error: " << sts);                    
                });
            });

            _requests = std::vector<rs_usb_request>(_context.request_count);
            LOG_DEBUG("uvc_streamer::init request_count= "  << (uint16_t) _context.request_count << "requesr size = " << _requests.size());
            for(auto&& r : _requests)
            {
                r = _context.messenger->create_request(_read_endpoint);
                r->set_buffer(std::vector<uint8_t>(_read_buff_length));
                r->set_callback(_request_callback);
            }
            LOG_DEBUG("uvc_streamer::init end-----");
        }

        void uvc_streamer::start()
        {
            LOG_DEBUG("uvc_streamer::start");
            _action_dispatcher.invoke_and_wait([this](dispatcher::cancellable_timer c)
            {
                if(_running)
                    return;

                auto inf = _context.usb_device->get_interface_isoc(_context.control->bInterfaceNumber , _context.control->dwMaxPayloadTransferSize );
                if (!inf)
                _context.messenger->reset_endpoint(_read_endpoint, RS2_USB_ENDPOINT_DIRECTION_READ);

                {
                    std::lock_guard<std::mutex> lock(_running_mutex);
                    _running = true;
                }

                for(auto&& r : _requests)
                {
                    auto sts = _context.messenger->submit_request(r);
                    if(sts != platform::RS2_USB_STATUS_SUCCESS)
                        throw std::runtime_error("failed to submit UVC request while start streaming");
                }

                _publish_frame_thread->start();

            }, [this](){ return _running; });
        }

        void uvc_streamer::stop()
        {  LOG_DEBUG("uvc_streamer::stop " << __LINE__ << " " << __FUNCTION__ );
            _action_dispatcher.invoke_and_wait([this](dispatcher::cancellable_timer c)
            {
                if(!_running)
                    return;



                _request_callback->cancel();
                LOG_DEBUG("------- _request_callback  cancel end---- frame = " << _frames_archive->get_size() );
                _watchdog->stop();

                _frames_archive->stop_allocation();

                _queue.clear();


                int i = 0;
                for(auto&& r : _requests) {
                    LOG_DEBUG("count " << (int) i++);
                    _context.messenger->cancel_request(r);
                }
                LOG_DEBUG("--------------free Cancel request " << _requests.size() <<  "count= " << i );

                _requests.clear();

                auto inf = _context.usb_device->get_interface_isoc(_context.control->bInterfaceNumber , _context.control->dwMaxPayloadTransferSize );
                if (inf) // close stream
                    _context.messenger->set_interface(inf->get_number(), 0);

                if (!inf) //  randy debug prevent hang in isoc mode
                {
                    _frames_archive->wait_until_empty();
                    LOG_DEBUG("--- reset endpoint ");
                    _context.messenger->reset_endpoint(_read_endpoint,
                                                       RS2_USB_ENDPOINT_DIRECTION_READ);
                }
                _publish_frame_thread->stop();


                {
                    std::lock_guard<std::mutex> lock(_running_mutex);
                    _running = false;
                    _stopped_cv.notify_one();
                }

            }, [this](){ return !_running; });
            LOG_DEBUG("uvc_streamer::end");
        }

        void uvc_streamer::flush()
        {
            if(_running)
                stop();

            // synchronized so do not destroy shared pointers while it's still being running
            {
                std::unique_lock<std::mutex> lock(_running_mutex);
                _stopped_cv.wait_for(lock, std::chrono::seconds(1), [&]() { return !_running; });
            }

            _read_endpoint.reset();

            _watchdog.reset();
            _publish_frame_thread.reset();
            _request_callback.reset();

            _frames_archive.reset();

            _action_dispatcher.stop();
        }

        bool uvc_streamer::wait_for_first_frame(uint32_t timeout_ms)
        {
            auto start = std::chrono::system_clock::now();
            while(!_frame_arrived)
            {
                auto end = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                if(duration > timeout_ms)
                    break;
            }
            return _frame_arrived;
        }
    }
}
