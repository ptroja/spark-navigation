with Algorithm;
with Interfaces.C; use Interfaces;

package Algorithm.Alloc is

   -- C interface.
   type VFH_Ptr is access Algorithm.VFH;

   function Create_VFH_Ptr
     (cell_size                     : C.C_float;
      window_diameter               : C.int;
      sector_angle                  : C.int;
      safety_dist_0ms               : C.C_float;
      safety_dist_1ms               : C.C_float;
      max_speed                     : C.int;
      max_speed_narrow_opening      : C.int;
      max_speed_wide_opening        : C.int;
      max_acceleration              : C.int;
      min_turnrate                  : C.int;
      max_turnrate_0ms              : C.int;
      max_turnrate_1ms              : C.int;
      min_turn_radius_safety_factor : C.C_float;
      free_space_cutoff_0ms         : C.C_float;
      obs_cutoff_0ms                : C.C_float;
      free_space_cutoff_1ms         : C.C_float;
      obs_cutoff_1ms                : C.C_float;
      weight_desired_dir            : C.C_float;
      weight_current_dir            : C.C_float)
      return VFH_Ptr;
   pragma Export(CPP, Create_VFH_Ptr);

   procedure Destroy_VFH_Ptr(This : in out VFH_Ptr);
   pragma Export(CPP, Destroy_VFH_Ptr);

end;
