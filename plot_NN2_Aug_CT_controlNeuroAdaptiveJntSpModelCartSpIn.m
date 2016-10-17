%%
clear all; close all; clc;
load('sim.mat')

[tau_h,e]= robout2layerJntSpModelCartSpIn( t, x, z_mdl, z_sta, z_t );