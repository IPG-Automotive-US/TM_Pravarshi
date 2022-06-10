## **********************************************************************
## FileCreator 	TruckMaker/Rigid_Trucktest011218.tcl 2018-06-12 08:23:18
## FileName 	Rigid Truck
## Description: Generated - not validated - Rigid Truck
## 		FileCreator TruckMaker/Vehicle Data Set Generator 2018-06-12 08:23:18
## 		driveline Gen4x2
## 		engine pnom 240.0 kW (326 PS) at 1800.0 rpm
## 		max. torque 1700.0 Nm at 1100.0 rpm
## 		unloaded weight 7000.0 kg
## 		loaded weight 18000.0 kg
## 		length x width x height 11010.0 mm x 2495.0 mm x 3365.0 mm
## 		wheel base 9224.0 mm
## 		track width front 2200.0 mm
## 		track width rear 2200.0 mm
## 		front overhang 1365.0 mm
## 		rear overhang 1100.0 mm
## 		tire 295/80 R 22.5
## 		body RigidBody
## 		platform off
## 		cab off
## 		no of axle 2.0
## **********************************************************************
proc DefineCarSide {cv {col gray} {ShowDetails 0}} {
    $cv create line 1101.0 51.8 1101.0 168.25 -fill $col 
    $cv create line 1101.0 168.25 1081.0 336.5 -fill $col 
    $cv create line 1081.0 336.5 947.4 336.5 -fill $col 
    $cv create line 947.4 336.5 947.4 112.17 -fill $col 
    $cv create line 947.4 112.17 0.0 112.17 -fill $col 
    $cv create line 0.0 112.17 0.0 336.5 -fill $col 
    $cv create line 0.0 336.5 947.4 336.5 -fill $col 
    $cv create line 0.0 112.17 0.0 51.8 -fill $col 
    $cv create line 0.0 51.8 -9.7 51.8 -fill $col 
    $cv create line -14.7 51.8 -12.765 66.501 -fill $col 
    $cv create line -12.765 66.501 -7.09 80.2 -fill $col 
    $cv create line -7.09 80.2 1.936 91.964 -fill $col 
    $cv create line 1.936 91.964 13.7 100.99 -fill $col 
    $cv create line 13.7 100.99 27.399 106.67 -fill $col 
    $cv create line 27.399 106.67 42.1 108.6 -fill $col 
    $cv create line 42.1 108.6 56.801 106.67 -fill $col 
    $cv create line 56.801 106.67 70.5 100.99 -fill $col 
    $cv create line 70.5 100.99 82.264 91.964 -fill $col 
    $cv create line 82.264 91.964 91.29 80.2 -fill $col 
    $cv create line 91.29 80.2 96.965 66.501 -fill $col 
    $cv create line 96.965 66.501 98.9 51.8 -fill $col 
    $cv create line 93.9 51.8 912.7 51.8 -fill $col 
    $cv create line 907.7 51.8 909.63 66.501 -fill $col 
    $cv create line 909.63 66.501 915.31 80.2 -fill $col 
    $cv create line 915.31 80.2 924.34 91.964 -fill $col 
    $cv create line 924.34 91.964 936.1 100.99 -fill $col 
    $cv create line 936.1 100.99 949.8 106.67 -fill $col 
    $cv create line 949.8 106.67 964.5 108.6 -fill $col 
    $cv create line 964.5 108.6 979.2 106.67 -fill $col 
    $cv create line 979.2 106.67 992.9 100.99 -fill $col 
    $cv create line 992.9 100.99 1004.7 91.964 -fill $col 
    $cv create line 1004.7 91.964 1013.7 80.2 -fill $col 
    $cv create line 1013.7 80.2 1019.4 66.501 -fill $col 
    $cv create line 1019.4 66.501 1021.3 51.8 -fill $col 
    $cv create line 1016.3 51.8 1101.0 51.8 -fill $col 
    $cv create line 1091.0 173.25 1074.0 326.5 -fill $col 
    $cv create line 1074.0 326.5 1016.3 326.5 -fill $col 
    $cv create line 1016.3 326.5 1016.3 173.25 -fill $col 
    $cv create line 1016.3 173.25 1091.0 173.25 -fill $col 
    $cv create oval		-9.7 0.0 93.9 103.6 -fill gray -outline $col
    $cv create oval		13.9 23.6 70.3 80.0 -fill white -outline $col
    $cv create oval		912.7 0.0 1016.3 103.6 -fill gray -outline $col
    $cv create oval		936.3 23.6 992.7 80.0 -fill white -outline $col
}
proc DefineCarTop {cv {col gray} {ShowDetails 0}} {
    $cv create line 1101.0 -124.75 1081.0 -124.75 -fill $col 
    $cv create line 1081.0 -124.75 947.4 -124.75 -fill $col 
    $cv create line 1101.0 -124.75 1101.0 124.75 -fill $col 
    $cv create line 1081.0 -124.75 1081.0 124.75 -fill $col 
    $cv create line 947.4 -124.75 947.4 124.75 -fill $col 
    $cv create line 1101.0 124.75 1081.0 124.75 -fill $col 
    $cv create line 1081.0 124.75 947.4 124.75 -fill $col 
    $cv create line 947.4 -124.75 0.0 -124.75 -fill $col 
    $cv create line 0.0 -124.75 0.0 124.75 -fill $col 
    $cv create line 947.4 124.75 0.0 124.75 -fill $col 
    $cv create line 1091.0 -119.75 1091.0 119.75 -fill $col 
    $cv create line 1091.0 119.75 1081.0 119.75 -fill $col 
    $cv create line 1081.0 119.75 1081.0 -119.75 -fill $col 
    $cv create line 1081.0 -119.75 1091.0 -119.75 -fill $col 
    $cv create rectangle	-9.7 -124.75 93.9 -95.75 -fill gray -outline $col
    $cv create rectangle	-9.7 124.75 93.9 95.75 -fill gray -outline $col
    $cv create rectangle	912.7 -124.75 1016.3 -95.75 -fill gray -outline $col
    $cv create rectangle	912.7 124.75 1016.3 95.75 -fill gray -outline $col
    $cv create rectangle 	-9.7 -94.75 93.9 -65.75 -fill gray -outline $col
    $cv create rectangle	-9.7 94.75 93.9 65.75 -fill gray -outline $col
}
