package com.intel.realsense.librealsense;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.util.Log;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class UsbUtilities {
    private static final String TAG = "librs UsbUtilities";
    public static final String ACTION_USB_PERMISSION = "USB_CONTROL_PERMISSION";

    private static final Boolean isUseRobotEye = true;

    public static boolean isIntel(UsbDevice usbDevice){
		//abt
        if (usbDevice.getVendorId() == 0x8086 || usbDevice.getVendorId() == 0x143C || usbDevice.getVendorId() == 0x05a6)
            return true;

        return false;
    }

    private static List<UsbDevice> getUsbDevices(Context context, Integer vId) {
        //abt
        List<UsbDevice> res;
        if (vId == 0x143c) {
            res = getUsbDevices(context, vId, 0x99AA);
            if(res.isEmpty())
                res = getUsbDevices(context, vId, 0x99BB);
        }
        else
            res =  getUsbDevices(context, vId, 0);

        if(res.isEmpty())
            Log.w("abt", "getUsbDevices: not found");

        return res;
    }

    private static List<UsbDevice> getUsbDevices(Context context, Integer vId, Integer pId) {
        ArrayList<UsbDevice> res = new ArrayList<>();
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);

        Log.w("abt", "get usb devices PID: 0x"+Integer.toHexString(pId)+", VID: 0x"+Integer.toHexString(vId));

        HashMap<String, UsbDevice> devicesMap = usbManager.getDeviceList();
        for (Map.Entry<String, UsbDevice> entry : devicesMap.entrySet()) {
            UsbDevice usbDevice = entry.getValue();
            Log.d(TAG, "usbDevice.getProductId() = 0x"+ Integer.toHexString(usbDevice.getProductId()));
            Log.d(TAG, "usbDevice.getVendorId() = 0x"+ Integer.toHexString(usbDevice.getVendorId()));
            if(usbDevice.getVendorId() == vId && (usbDevice.getProductId() == pId || pId == 0))
            {
                res.add(usbDevice);
            }
        }
        if (res.isEmpty())
            Log.w(TAG, "getUsbDevice: failed to locate USB device, " + "VID: " + String.format("0x%04x", vId) + ", PID: " + String.format("0x%04x", pId));
        return res;
    }

    private static boolean hasUsbPermission(Context context, UsbDevice usbDevice){
        Log.d(TAG, "hasUsbPermission");

        if(usbDevice == null){
            Log.w(TAG, "hasUsbPermission: null USB device");
            return false;
        }

        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        return usbManager.hasPermission(usbDevice);
    }

    private static void grantUsbPermissions(Context context, UsbDevice usbDevice){
        Log.d(TAG, "grantUsbPermissions");

        if(usbDevice == null){
            Log.w(TAG, "grantUsbPermissions: null USB device");
            return;
        }

        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        boolean permission = usbManager.hasPermission(usbDevice);

        if(!permission) {
            Log.i(TAG, "grantUsbPermissions:\ndevice: " + usbDevice.toString());
            PendingIntent pi = PendingIntent.getBroadcast(context, 0, new Intent(UsbUtilities.ACTION_USB_PERMISSION), 0);
            usbManager.requestPermission(usbDevice, pi);
        }
    }

    private static List<UsbDevice> getDevices(Context context) {
        //abt
        List<UsbDevice> res = getUsbDevices(context, 0x143C);
        if (res.isEmpty())
            res = getUsbDevices(context, 0x05a6);
        if (res.isEmpty())
            res = getUsbDevices(context, 0x8086);
        if(res.isEmpty())
            Log.w("abt", "getDevices: not found");
        return res;
    }

    public static void grantUsbPermissionIfNeeded(Context context) {
        List<UsbDevice> usbDevices = getDevices(context);
        for (UsbDevice usbDevice : usbDevices) {
            if (!hasUsbPermission(context, usbDevice)) {
                grantUsbPermissions(context, usbDevice);
            }
        }
    }
}
