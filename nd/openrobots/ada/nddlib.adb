with nddLibUtils; use nddLibUtils;

package body nddLib is

   -----------------------
   -- nddIsNarrowValley --
   -----------------------

   function nddIsNarrowValley
     (v : NDD_VALLEY;
      max_n_sectors : Integer;
      smax : Integer)
      return NDD_BOOL
   is
      t : Integer;
   begin
        t := v.end_idx - v.begin_idx;
      if (t < 0) then t := t + max_n_sectors;
      end if;
      if (t < smax) then
         return True;
      end if;
      return False;
   end nddIsNarrowValley;

   ---------------------
   -- nddIsWideValley --
   ---------------------

   function nddIsWideValley
     (v : NDD_VALLEY;
      max_n_sectors : Integer;
      smax : Integer)
      return NDD_BOOL
   is
   begin
      return not nddIsNarrowValley(v, max_n_sectors, smax);
   end nddIsWideValley;

   ----------------
   -- nddFillRnd --
   ----------------

   procedure nddFillRnd
     (diag : in out NDD_DIAGRAM;
      enlargement : in NDD_SECTOR_RANGES)
   is
   begin
      -- for (i = 0; i<diag->nsectors; i++) {
      for i in diag.pnd'Range loop
         if (diag.pnd(i) = NDDLIB_PND_FREE) then
            diag.rnd(i) := NDDLIB_PND_FREE;
         elsif (diag.pnd(i) = NDDLIB_PND_OCCUPIED) then
               diag.rnd(i) := diag.dmax_sensor
		     - nddEstimateDistToObsFromPND(i, diag)
		     + enlargement(i);
         else
            diag.rnd(i) := diag.pnd(i) - diag.lmax_robot + enlargement(i);
         end if;
      end loop;
   end nddFillRnd;

   function nddFindValleys(diag : in out NDD_DIAGRAM) return Integer
   is
      modulo : constant Positive := diag.nsectors;
      i : Integer := 0;
      j : Integer := 1;
      firstI, firstJ : Integer;
      begin_idx, end_idx : Integer;
      n_valley : Integer := 0;
      testJ : Integer := -1;
      currentState : NDD_FINDVALLEYS := NDD_FINDVALLEYS_START;
   begin

      while currentState /= NDD_FINDVALLEYS_END loop

      case currentState is
      when NDD_FINDVALLEYS_START =>
         if (diag.pnd(i) = NDDLIB_PND_OCCUPIED) then
            incIincJ(i, j, modulo);
            if (j = 0) then
               -- fprintf(stderr, "nddLib: zero points from findValleys\n");
               currentState := NDD_FINDVALLEYS_END;
            end if;
         else
            currentState := NDD_FINDVALLEYS_GOT_I;
         end if;

      when NDD_FINDVALLEYS_GOT_I =>
         if (j=0) then
            -- fprintf(stderr, "nddLib: not enough disc from findValleys\n");
            currentState := NDD_FINDVALLEYS_END;
         elsif (diag.pnd(j) = NDDLIB_PND_OCCUPIED) then
            incJ(j,modulo);
         else
            currentState := NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC;
         end if;
         when NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC =>
      if (loweringDisc(i,j,diag)) then
	firstI := i;
	firstJ := j;
	currentState := NDD_FINDVALLEYS_ONE_BEGIN_DISC;
      else
	incIincJ(i, j, modulo);
	currentState := NDD_FINDVALLEYS_GOT_I;
      end if;
    when NDD_FINDVALLEYS_ONE_BEGIN_DISC =>
      begin_idx := i;
      currentState := NDD_FINDVALLEYS_BEGIN_DISC_MARKED;
    when NDD_FINDVALLEYS_BEGIN_DISC_MARKED =>
      incIincJ(i, j, modulo);
      currentState := NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC;
    when NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC =>
      if (diag.pnd(j) = NDDLIB_PND_OCCUPIED) then
	incJ(j,modulo);
      else
	currentState := NDD_FINDVALLEYS_GOT_J_FOR_END_DISC;
      end if;
    when NDD_FINDVALLEYS_GOT_J_FOR_END_DISC =>
      if ( (i=firstI) and (j=firstJ) ) then
	diag.valleys(0).begin_idx := begin_idx;
	currentState := NDD_FINDVALLEYS_END;
      elsif (raisingDisc(i,j,diag)) then
	end_idx := j;
	currentState := NDD_FINDVALLEYS_END_DISC_MARKED;
      elsif ( (testJ = j) and (n_valley = 0)) then
	-- fprintf(stderr, "nddLib: cannot find any raising disc\n");
	currentState := NDD_FINDVALLEYS_END;
      else
               currentState := NDD_FINDVALLEYS_BEGIN_DISC_MARKED;
            end if;
      if (testJ = -1) then testJ := j; end if;

    when NDD_FINDVALLEYS_END_DISC_MARKED =>
      incIincJ(i, j, modulo);
      currentState := NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC;
    when NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC =>
      if (diag.pnd(j) = NDDLIB_PND_OCCUPIED) then
	incJ(j,modulo);
      else
	currentState := NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC;
      end if;
    when NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC =>
      if (loweringDisc(i,j,diag)) then
	diag.valleys(n_valley).end_idx := end_idx;
               diag.valleys(n_valley).begin_idx := begin_idx;
               n_valley := n_valley + 1;
	currentState := NDD_FINDVALLEYS_LOWERING_DISC_DETECTED;
      else
	if (raisingDisc(i,j,diag)) then
                  end_idx := j;
                  end if;
	currentState := NDD_FINDVALLEYS_END_DISC_MARKED;
      end if;
    when NDD_FINDVALLEYS_LOWERING_DISC_DETECTED =>
      if ( (i = firstI) and (j = firstJ) ) then
	currentState := NDD_FINDVALLEYS_END;
      else
	currentState := NDD_FINDVALLEYS_ONE_BEGIN_DISC;
      end if;
    when NDD_FINDVALLEYS_END =>
      null;
    end case;
  end loop;

      if (n_valley = 0) then
            diag.valleys(0).begin_idx := modulo/2+1;
            diag.valleys(0).end_idx := modulo/2;
--  #ifdef VERBOSE_NDDLIB
--      fprintf(stdout, "nddLib: one artificial valley: %d %d\n",
--  	    diag->valleys[0].begin, diag->valleys[0].end);
--  #endif
            n_valley := 1;
      end if;

      diag.n_valleys := n_valley;
      return n_valley;
   end nddFindValleys;

end nddLib;
