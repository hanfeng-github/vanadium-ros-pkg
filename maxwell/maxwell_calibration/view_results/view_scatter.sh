#! /bin/bash
rosrun calibration_estimation post_process.py /tmp/maxwell_calibration/cal_measurements.bag /tmp/maxwell_calibration/ `rospack find maxwell_calibration`/view_results/scatter_config.yaml
