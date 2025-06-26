% Test script for augment_trajectory function
clc; close all;

timestamp = '25-06-24-21-30-34';
augmentation_id = 1;

% Test trajectory transformation
options.visualize = false;
options.simulate = true;
options.save_result = false;

% Call the augment_trajectory function
augment_trajectory(timestamp, augmentation_id, options);