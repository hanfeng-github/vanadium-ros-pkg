#! /bin/bash

original_system=`rospack find maxwell_calibration`/estimate_params/config/system.yaml
final_system=/tmp/maxwell_calibration/system_calibrated.yaml
bag_in=/tmp/maxwell_calibration/cal_measurements.bag
urdf_out=robot_calibrated.xml
`rospack find maxwell_calibration`/write_urdf/update_maxwell_urdf.py $original_system $final_system $bag_in $urdf_out #> /tmp/maxwell_calibration/urdf_writer_debug.log
if [ $? = 0 ]
then
    echo "Wrote new URDF to $urdf_out"
else
    echo "Error writing URDF"
fi

