#ifndef _INTERFACE_H
#define _INTERFACE_H

extern "C" {

  void vfhinit(void);
  void vfhfinal(void);
   
  void * create_vfh_ptr(
			float cell_size,
			int window_diameter,
			int sector_angle,
			float safety_dist_0ms,
			float safety_dist_1ms, 
			int max_speed,
			int max_speed_narrow_opening,
			int max_speed_wide_opening,
			int max_acceleration,
			int min_turnrate,
			int max_turnrate_0ms,
			int max_turnrate_1ms,
			float min_turn_radius_safety_factor,
			float free_space_cutoff_0ms,
			float obs_cutoff_0ms,
			float free_space_cutoff_1ms,
			float obs_cutoff_1ms,
			float weight_desired_dir,
			float weight_current_dir );

  void destroy_vfh_ptr(void * vfh);
  
  void Init(void * vfh);
    
  void Update_VFH(void * vfh,
		  float laser_ranges[361][2], 
		  int current_speed,  
		  float goal_direction,
		  float goal_distance,
		  float goal_distance_tolerance,
		  int &chosen_speed, 
		  int &chosen_turnrate );

  int GetMinTurnrate(void * vfh);

  int GetMaxTurnrate(void * vfh, int speed);
  int GetCurrentMaxSpeed(void * vfh);
  
  void SetRobotRadius(void * vfh, float robot_radius );
  void SetMinTurnrate(void * vfhThis, int min_turnrate);
  void SetCurrentMaxSpeed(void * vfh, int Max_Speed);
  
} /* extern "C" */

#endif /* _INTERFACE_H */
