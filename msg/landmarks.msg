#v1************************************************************************************
# This is developed at Carnegie Mellon University in collaboration with Autel Robotics 
#                                                                                      
# PI:                                                                                  
# George Kantor                                                                        
#                                                                                      
# Authors:                                                                              
# Weizhao Shao                                                                         
# Cong Li                                                                              
# Srinivasan Vijayarangan                                                              
#                                                                                      
# Please refer to the contract document for details on license/copyright information.  
#**************************************************************************************
std_msgs/Header header
uint32 frame_id
uint32 size
uint32[] feature_ids
float64[] leftPixel_xs
float64[] leftPixel_ys
float64[] location_x
float64[] location_y
float64[] location_z
