with Ada.Numerics;
with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;

with nddLib; use nddLib;

package body nddLibUtils is

   M_PI : constant Float := Ada.Numerics.Pi;

   function nddEstimateDistToObsFromPND(i : in Integer;
                                        diag : in out NDD_DIAGRAM) return Float
   is
      j,k : Integer;
      nsectors : constant Integer := diag.nsectors;
      dist1, dist2, dist : Float;
      r0, r1, r2 : Float;
      x1, x2, y1, y2, x, y : Float;
      a,b : Float;
      p : Float;
   begin

      if (diag.pnd(i) /= NDDLIB_PND_OCCUPIED) then
         return diag.dmax_sensor + diag.lmax_robot - diag.pnd(i);
      end if;

      j := i+1;
      if (j = nsectors) then j := 0; end if;
      while (diag.pnd(j) = NDDLIB_PND_OCCUPIED) loop
         j := j + 1;
         if (j = nsectors) then j := 0; end if;
      end loop;
      k := i-1;
      if (k < 0) then k := nsectors-1; end if;
      while (diag.pnd(k) = NDDLIB_PND_OCCUPIED) loop
         k := k - 1;
         if (k < 0) then k := nsectors-1; end if;
      end loop;

      dist1 := diag.dmax_sensor + diag.lmax_robot - diag.pnd(j);
      dist2 := diag.dmax_sensor + diag.lmax_robot - diag.pnd(k);

      r0 := (Float (i) + 0.5) * 2.0 * M_PI / Float(nsectors);
      if (r0>M_PI) then r0 := r0 - 2.0*M_PI; end if;
      r1 := (Float(j) + 0.5) * 2.0 * M_PI / Float(nsectors);
      r2 := (Float(k) + 0.5) * 2.0 * M_PI / Float(nsectors);

      x1 := dist1 * cos(r1);
      x2 := dist2 * cos(r2);
      y1 := dist1 * sin(r1);
      y2 := dist2 * sin(r2);
      p := tan(r0);

      if (abs(x1-x2) > NDDLIB_EPSILON_ESTIMATE_FROM_PND) then
         a := (y1-y2)/(x1-x2);
         if (y1<y2) then
            b := y1 - a*x1;
         else
            b := y2 - a*x2;
         end if;
         x := b / (p-a);
      elsif (x1<x2) then
         x := x1;
      else
         x := x2;
      end if;
      y := p * x;
      dist := sqrt(x*x+y*y);
      return dist;
   end nddEstimateDistToObsFromPND;

   function nddSectorDistance (d1, d2, n : Integer) return Integer
   is
      ret : Integer;
   begin
      ret := abs(d1 - d2);
      if (ret > (n/2)) then
         return (n - ret);
      else
         return ret;
      end if;
   end nddSectorDistance;

   -- findValleys utils

   procedure incJ(j : in out Integer; modulo : in Integer) is
   begin
      j := j + 1;
      if ( j = modulo ) then
         j := 0;
      end if;
   end incJ;

   procedure incIincJ(i, j : in out Integer; modulo : in Integer) is
   begin
      i := j;
      incJ(j, modulo);
   end;

   function loweringDisc (i, j : in Integer; diag : in NDD_DIAGRAM) return Integer
   is
   begin
      if ( (diag.pnd(i) - diag.pnd(j)) > diag.lmax_robot) then
         return 1;
      else
         return 0;
      end if;
   end loweringDisc;

   function raisingDisc (i, j : in Integer; diag : in NDD_DIAGRAM) return Integer
   is
   begin
      if ((diag.pnd(i) - diag.pnd(j)) < (-diag.lmax_robot)) then
         return 1;
      else
         return 0;
      end if;
   end raisingDisc;

   -- splitValleys utils

   procedure nddSplitNValleys(diag : in out NDD_DIAGRAM;
                              valleyToSplit : in Integer;
                              width : in Integer;
                              nSlices : in Integer)
   is
      dum : Integer;
      se : constant Integer := diag.valleys(valleyToSplit).end_idx;
      nsects : constant Integer := diag.nsectors;
   begin
      dum := diag.valleys(valleyToSplit).begin_idx + width;
      if (dum >= nsects) then dum := dum - nsects; end if;
      diag.valleys(valleyToSplit).end_idx := dum;

      for j in 1 .. nSlices-1 loop
         diag.valleys(diag.n_valleys).begin_idx := dum;
         dum := dum + width;
         if (dum >= nsects) then dum := dum - nsects; end if;
         diag.valleys(diag.n_valleys).end_idx := dum;
         diag.n_valleys := diag.n_valleys + 1;
      end loop;

      diag.valleys(diag.n_valleys).begin_idx := dum;
      diag.valleys(diag.n_valleys).end_idx := se;
      diag.n_valleys := diag.n_valleys + 1;
   end nddSplitNValleys;

   function nddHowManySplits(width, nsectors, maxFrac : Integer) return Integer
   is
   begin
      for i in maxFrac-1 .. 1 loop
         if ( Float(width)>=(Float(i)*Float(nsectors)/Float(maxFrac) )) then
            return i;
         end if;
      end loop;
      return 0;
   end nddHowManySplits;

   -- ...

   function nddGiveSpeedFactorFromAngle(theta : in Float;
                                        linFactor : in Positive) return Float
   is
      factor : Float := 1.0;
   begin
      for i in 0 .. linFactor-1 loop
         factor := factor * NDDLIB_LIMIT_ANGLE * theta / M_PI;
      end loop;
      return factor;
   end nddGiveSpeedFactorFromAngle;

end nddLibUtils;
