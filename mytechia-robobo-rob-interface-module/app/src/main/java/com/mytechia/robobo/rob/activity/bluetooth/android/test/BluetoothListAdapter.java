package com.mytechia.robobo.rob.activity.bluetooth.android.test;

import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.CheckedTextView;


import java.util.List;

/**
 * Created by julio on 1/10/15.
 */
public class BluetoothListAdapter extends ArrayAdapter<BluetoothDevice> {


    public BluetoothListAdapter(Context context, int resource, List<BluetoothDevice> bluetoothDeviceList) {
        super(context, resource,  bluetoothDeviceList);
    }

    @Override
    public View getView(int position, View rowView, ViewGroup parent){

        LayoutInflater inflater = (LayoutInflater) super.getContext().getSystemService(Context.LAYOUT_INFLATER_SERVICE);

        if(rowView==null) {
            rowView = (ViewGroup) inflater.inflate(R.layout.bluetooth_device_list, parent, false);
        }

        ViewGroup rowViewGroup= (ViewGroup)rowView;

        CheckedTextView checkedTextView= (CheckedTextView) rowViewGroup.findViewById(R.id.checkedBluetoothDevice);

        BluetoothDevice bluetoothDevice=super.getItem(position);


        checkedTextView.setText(bluetoothDevice.getName() + " " + bluetoothDevice.getAddress());


        return rowView;
    }
}
