with nddLib;
with nddStruct; use nddStruct;

-- FIXME: single-linked list would suffice.
with Ada.Containers.Doubly_Linked_Lists;

package nddLibUtils is

   -- TODO: discriminant union?
   NDDLIB_PND_OCCUPIED : constant Float := -1.0;
   NDDLIB_PND_FREE : constant Float := 0.0;

   type NDD_FINDVALLEYS is (
                            NDD_FINDVALLEYS_START,
                            NDD_FINDVALLEYS_GOT_I,
                            NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC,
                            NDD_FINDVALLEYS_ONE_BEGIN_DISC,
                            NDD_FINDVALLEYS_BEGIN_DISC_MARKED,
                            NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC,
                            NDD_FINDVALLEYS_GOT_J_FOR_END_DISC,
                            NDD_FINDVALLEYS_END_DISC_MARKED,
                            NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC,
                            NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC,
                            NDD_FINDVALLEYS_LOWERING_DISC_DETECTED,
                            NDD_FINDVALLEYS_END);

   -- List of floats.
   package FloatList is new Ada.Containers.Doubly_Linked_Lists(Float);
   subtype nddFloatList is FloatList.List;

   function nddSectorDistance (d1, d2, n : Integer) return Integer;

   function nddEstimateDistToObsFromPND(i : in Integer;
                                        diag : in out NDD_DIAGRAM) return Float;

--  double nddEstimateDistToObstFromValues(nddFloatList* values_list);

   procedure incJ(j : in out Integer; modulo : in Integer);
   procedure incIincJ(i, j : in out Integer; modulo : in Integer);
   function loweringDisc (i, j : in Integer; diag : in NDD_DIAGRAM) return Integer;
   function raisingDisc (i, j : in Integer; diag : in NDD_DIAGRAM) return Integer;

   procedure nddSplitNValleys(diag : in out NDD_DIAGRAM;
                              valleyToSplit : in Integer;
                              width : in Integer;
                              nSlices : in Integer);

   function nddHowManySplits(width, nsectors, maxFrac : Integer) return Integer;
--  void nddGetBbAndBe(int sb, int se, int nsects, int *bb, int *be);
--
--  void nddGetSiAndSjFromSelectedValley (NDD_DIAGRAM * diag, int*si, int*sj);
--  void nddGetSiAndSjFromSector(NDD_DIAGRAM * diag, int sector, int*si, int*sj);
--
--  double nddGiveAngleFromSector(NDD_SECTOR sector, int nSectors);
--
--  NDD_SEGMENT * nddFiltrateSegments(NDD_SEGMENT * segments, int n_segs, double dmax, int * newNSegs);
--
--  int nddComputeAlpha(double lmax, double secuDist, double dDisc, int nsects);
--  int nddComputeGamma(double dObs, int sectObs,
--  		    int sectDisc, double dEnlargement,
--  		    int alpha, int nsects);
--  int nddComputeBeta(double dObs, int discObs,
--  		   int discGoal, double dEnlargement, int nsects);
--
--  int nddGetBissector(int s1, int s2, int nsects);
--
--  int nddIsGoalInValley(NDD_DIAGRAM * diag, NDD_POINT * goal);
--
--  int nddGetGoalSector(NDD_POINT * goal, int nsectors);
--
--
--  double nddGiveEnhacedSecurityDistance(int sector, int nSectors,
--  				      double secuDist, double lmax_robot);
--

   function nddGiveSpeedFactorFromAngle(theta : in Float;
                                        linFactor : in Positive) return Float;

end nddLibUtils;
