#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "nddLib.h"
#include "nddLibPrivate.h"
#include "nddLibUtils.h"


static float nddEstimateValleyWidth(int s1, int s2, NDD_DIAGRAM * diag,
				    double *d1, double *d2, double *ang);


float * nddComputeEnlargement(int nSect, float lmaxRobot, float secuDist,
 			      float vmaxRobot, float timeStep) { 
   int i;  
   float * e;  
   double alpha; 
   float enlargement; 
   float sectSize = 2.0 * M_PI / (float) nSect; 
   double ensd;;
     
   e = (float *) malloc (nSect*sizeof(float)); 
  
   for (i=0; i<nSect; i++) 
     e[i] = lmaxRobot*0.75; 

   for (i = 0; i<(nSect/4)-1; i++) { 
     alpha = (float) i * sectSize + sectSize/2.0;  
     enlargement = fabs (vmaxRobot*timeStep  
			 - (float) atan(M_PI/2.0-alpha)/atan(M_PI/2.0));    
     ensd = nddGiveEnhacedSecurityDistance(i,nSect,secuDist, 
 					   lmaxRobot);
     e[i] = enlargement + ensd + lmaxRobot*0.25;  
     /*      e[i] = vmaxRobot*timeStep + lmaxRobot; */
     e[nSect-i-1] = e[i]; 
   } 
   return e;    
 } 



static int nddIsNarrowValley (NDD_VALLEY * v, int nsectors, int smax) {
  int t;
/*   t = v->begin - v->end; */
  t = v->end - v->begin;
  if (t < 0) t += nsectors;
  if (t < smax) return 1;
  return 0;
}

static int nddIsWideValley (NDD_VALLEY * v, int nsectors,int smax) { 
  return (1-nddIsNarrowValley (v,nsectors,smax));
}


static int nddFillPndFromPoints(NDD_DIAGRAM * diag, NDD_POINT * points, int n_pt,
			 double distMax) {
/*see IROS 2000 for an exhaustive explanation*/ 
/*   this list will contains all points in rho, theta, */
/*     next we compute the average for each sector, then we fill pnd */
  nddFloatList ** list_of_rho;
  int i, sector;
  int nsectors = diag->nsectors;
  double angle;
  double dist;
  nddFloatList * new_f;
  
  list_of_rho = (nddFloatList **) malloc 
    (nsectors * sizeof(nddFloatList*));
  
  for (i=0; i<nsectors; i++)
    list_of_rho[i] = NULL;

  /*for each cartesian point, fill our rho_list*/
  for (i=0; i<n_pt; i++) {
    if ( (points[i].x != 0.0) && (points[i].y != 0.0) ) { 
      angle = atan2(points[i].y, points[i].x);
      dist = sqrt (points[i].y*points[i].y + points[i].x*points[i].x);
      if (dist>=distMax+diag->lmax_robot/2.0)
	continue;
      sector = (int) ((double)angle  * (double)nsectors / 2.0 / M_PI);
      if (sector < 0) sector += nsectors;
      
      new_f = (nddFloatList *) malloc (sizeof(nddFloatList));
      new_f->next = list_of_rho[sector];
      new_f->v = dist;
      list_of_rho[sector] = new_f;
    }
  }
  
  for (i=0; i<nsectors; i++) {
    dist = nddEstimateDistToObstFromValues(list_of_rho[i]);
    nddFloatListFree(list_of_rho[i]);
    if (dist == NDDLIB_PND_OCCUPIED)
      diag->pnd[i] = NDDLIB_PND_OCCUPIED;
    else if (dist >= diag->dmax_sensor) 
	diag->pnd[i] = NDDLIB_PND_FREE;
    else if (dist < 0) {
      /* this case shouldnot be possible*/
      fprintf (stderr, "nddLib: nddFillPnd: dist<0 and != -1 !!!!!\n"); 
      diag->pnd[i] = NDDLIB_PND_FREE;
    } else if (dist == 0) 
      diag->pnd[i] = NDDLIB_PND_FREE;
    else 
      diag->pnd[i] = diag->dmax_sensor + diag->lmax_robot - dist;
    
  }
  
  free(list_of_rho);

  return nsectors;
}

static int nddFillPndFromSegments(NDD_DIAGRAM * diag, NDD_SEGMENT * segments, 
			   int n_segs, NDD_POINT * lonelyPoints, int n_lPoints) {
/*see IROS 2000 for an exhaustive explanation*/ 
/*   this list will contains all points in rho, theta, */
/*     next we compute the average for each sector, then we fill pnd */

  nddFloatList ** list_of_rho;
  int i,j , sector1, sector2;
  int nsectors = diag->nsectors;
  int dum;
  double dist;
  double  angle;
  nddFloatList * new_f;
  
  list_of_rho = (nddFloatList **) malloc 
    (nsectors * sizeof(nddFloatList*));
  
  for (i=0; i<nsectors; i++) list_of_rho[i] = NULL;

  /*for each polar point, fill our rho_list*/
  for (i=0; i<n_segs; i++) {
    
    sector1 = (int) (segments[i].t1  * (double)nsectors / 2.0 / M_PI);
    if (sector1 < 0)  sector1 += nsectors;
    else if (sector1 >=nsectors) sector1 -= nsectors;
    sector2 = (int) (segments[i].t2  * (double)nsectors / 2.0 / M_PI);
    if (sector2 < 0) sector2 += nsectors;
    else if (sector2 >=nsectors) sector2 -= nsectors;

    new_f = (nddFloatList *) malloc (sizeof(nddFloatList));
    new_f->next = list_of_rho[sector1];
    new_f->v = segments[i].r1;
    list_of_rho[sector1] = new_f;  
    new_f = (nddFloatList *) malloc (sizeof(nddFloatList));
    new_f->next = list_of_rho[sector2];
    new_f->v = segments[i].r2;
    list_of_rho[sector2] = new_f;  

    dum = sector2 - sector1;
    if (dum < 0) dum += nsectors;
    if (dum > 1) {
      j = sector1 + 1;    
      if (j >= nsectors) j -= nsectors;
      while (j != sector2) {
	new_f = (nddFloatList *) malloc (sizeof(nddFloatList));
	new_f->next = list_of_rho[j];
	new_f->v = NDDLIB_PND_OCCUPIED;
	list_of_rho[j++] = new_f;              
	if (j >= nsectors) j -= nsectors;
      }
    }
  }

  for (i=0; i<n_lPoints; i++) {     
    angle = atan2(lonelyPoints[i].y, lonelyPoints[i].x); 
    dist = sqrt (lonelyPoints[i].y * lonelyPoints[i].y +
		 lonelyPoints[i].x * lonelyPoints[i].x);
    sector1 = (int) ((double)angle  * (double)nsectors / 2.0 / M_PI);     
    if (sector1 < 0) sector1 += nsectors; 
    if (sector1 >= nsectors) sector1 -= nsectors;

    new_f = (nddFloatList *) malloc (sizeof(nddFloatList)); 
    new_f->next = list_of_rho[sector1]; 
    new_f->v = dist; 
    list_of_rho[sector1] = new_f;   
   } 
  
  for (i=0; i<nsectors; i++) {
    dist = nddEstimateDistToObstFromValues(list_of_rho[i]);
    nddFloatListFree(list_of_rho[i]);
    if (dist == NDDLIB_PND_OCCUPIED)
      diag->pnd[i] = NDDLIB_PND_OCCUPIED;
    else if (dist >= diag->dmax_sensor) 
      diag->pnd[i] = NDDLIB_PND_FREE;
    else if (dist < 0) {
      /* this case really shouldnot be possible */
      fprintf(stderr, "nddLib: nddFillPnd: dist<0 and != -1 : %f\n", dist);
      diag->pnd[i] = NDDLIB_PND_FREE;
    } else if (dist == 0) 
      diag->pnd[i] = NDDLIB_PND_FREE;
    else 
      diag->pnd[i] = diag->dmax_sensor + diag->lmax_robot - dist;
  }
  
  free(list_of_rho);

  return nsectors;
}

static void nddFillRnd(NDD_DIAGRAM * diag, float * enlargement) {
  int i;
  
  for (i = 0; i<diag->nsectors; i++) {
    if (diag->pnd[i] == NDDLIB_PND_FREE) 
      diag->rnd[i] = NDDLIB_PND_FREE;
    else if (diag->pnd[i] == NDDLIB_PND_OCCUPIED)
      diag->rnd[i] = diag->dmax_sensor 
		     - nddEstimateDistToObsFromPND(i, diag) 
		     + enlargement[i];
    else
      diag->rnd[i] = diag->pnd[i] - diag->lmax_robot + enlargement[i];
  }
}


static float nddEstimateValleyWidth(int s1, int s2, NDD_DIAGRAM * diag,
				    double *d1, double *d2, double *ang) {

  /* exported for debug purpose */
/*   double d1,d2; */  
/*   double angle; */
  *d1 = nddEstimateDistToObsFromPND(s1,diag);
  *d2 = nddEstimateDistToObsFromPND(s2,diag);
  
  *ang = (double) nddSectorDistance(s1,s2,diag->nsectors) - 1.0;
  *ang = (*ang) / (double)(diag->nsectors) * 2.0 * M_PI;
  
  return sqrt((*d1)*(*d1)+(*d2)*(*d2)-2*(*d1)*(*d2)*cos(*ang));
}


static int nddFindValleys(NDD_DIAGRAM * diag,  NDD_POINT * local_goal) {

  int modulo = diag->nsectors;
  int i = 0;
  int j = 1;
  int firstI, firstJ;
  int begin, end;
  int n_valley = 0;

  int testJ = -1;

  int currentState = NDD_FINDVALLEYS_START ;

  while (currentState != NDD_FINDVALLEYS_END)  {
    switch (currentState) {
    case NDD_FINDVALLEYS_START:			/* 1 */
      if (diag->pnd[i] == NDDLIB_PND_OCCUPIED) {
	incIincJ(&i, &j, modulo);
	if (j == 0) {
	  fprintf(stderr, "nddLib: zero points from findValleys\n"); 
	  currentState = NDD_FINDVALLEYS_END;
	  break;
	} 
      } else
 	currentState = NDD_FINDVALLEYS_GOT_I; 
      break; 
    case NDD_FINDVALLEYS_GOT_I:			/* 2 */
      if (j==0) {
	fprintf(stderr, "nddLib: not enough disc from findValleys\n"); 
	currentState = NDD_FINDVALLEYS_END;
      } else if (diag->pnd[j] == NDDLIB_PND_OCCUPIED) {
	incJ(&j,modulo);
      } else
	currentState = NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC;
      break;
    case NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC:	/* 3 */
      if (loweringDisc(i,j,diag)) {
	firstI = i;
	firstJ = j;
	currentState = NDD_FINDVALLEYS_ONE_BEGIN_DISC;
	break;
      } else {
	incIincJ(&i, &j, modulo);
	currentState = NDD_FINDVALLEYS_GOT_I;
	break;
      }
    case NDD_FINDVALLEYS_ONE_BEGIN_DISC:		/* 4 */
      begin = i;
      currentState = NDD_FINDVALLEYS_BEGIN_DISC_MARKED; 
      break; 
    case NDD_FINDVALLEYS_BEGIN_DISC_MARKED:	/* 5 */
      incIincJ(&i, &j, modulo);
      currentState = NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC; 
      break;  
    case NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC: /* 6 */
      if (diag->pnd[j] == NDDLIB_PND_OCCUPIED) {
	incJ(&j,modulo);
	break;
      }
      else  
	currentState = NDD_FINDVALLEYS_GOT_J_FOR_END_DISC; 
      break; 
    case NDD_FINDVALLEYS_GOT_J_FOR_END_DISC:	/* 7 */
      if ( (i==firstI) && (j==firstJ) ) {
	diag->valleys[0].begin = begin;
	currentState = NDD_FINDVALLEYS_END;
	break;
      }	
      if (raisingDisc(i,j,diag)) {
	end = j;
	currentState = NDD_FINDVALLEYS_END_DISC_MARKED;
	break;
      }
      if ( (testJ == j) && (n_valley == 0)) {
	fprintf(stderr, "nddLib: cannot find any raising disc\n");  
	currentState = NDD_FINDVALLEYS_END;
	break;
      }
      currentState = NDD_FINDVALLEYS_BEGIN_DISC_MARKED;  
      if (testJ == -1) 	testJ = j;      
      break;
    case NDD_FINDVALLEYS_END_DISC_MARKED:	/* 8 */
      incIincJ(&i, &j, modulo);
      currentState = NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC; 
      break; 
    case NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC: /* 9 */
      if (diag->pnd[j] == NDDLIB_PND_OCCUPIED) {	
	incJ(&j,modulo);
	break;
      } else 
	currentState = NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC;
      break;
    case NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC:	/* 10 */
      if (loweringDisc(i,j,diag)) {
	diag->valleys[n_valley].end = end;
	diag->valleys[n_valley++].begin = begin;
	currentState = NDD_FINDVALLEYS_LOWERING_DISC_DETECTED;
      } else {
	if (raisingDisc(i,j,diag)) 
	  end = j;
	currentState = NDD_FINDVALLEYS_END_DISC_MARKED;
      } 
      break;
    case NDD_FINDVALLEYS_LOWERING_DISC_DETECTED: /* 11 */
      if ( (i == firstI) && (j == firstJ) ) 
	currentState = NDD_FINDVALLEYS_END;
      else
	currentState = NDD_FINDVALLEYS_ONE_BEGIN_DISC;
      break;
    case NDD_FINDVALLEYS_END:
      break;			/* 12 */
    default:
      fprintf(stderr, "nddLib: nddFindValleys clash!\n");
    }
  }

  if (n_valley == 0) {
    diag->valleys[0].begin = modulo/2+1;
    diag->valleys[0].end = modulo/2;
#ifdef VERBOSE_NDDLIB
    fprintf(stdout, "nddLib: one artificial valley: %d %d\n",
	    diag->valleys[0].begin, diag->valleys[0].end);
#endif
    n_valley = 1;
  }
    
  diag->n_valleys = n_valley; 
  return n_valley; 
}


static int nddSplitValleys(NDD_DIAGRAM * diag) {
  int nSplit = 0;
  int i, sb, se;
  int nToTest = diag->n_valleys;
  int nsects = diag->nsectors;
  int split;
  int width;
  
  for (i=0; i<nToTest; i++) {
    sb = diag->valleys[i].begin;
    se = diag->valleys[i].end;

    if (sb == se) {
      /* avoid having same begin and end in rare cases (b=e=0) */
      diag->valleys[i].begin++;
      sb = diag->valleys[i].begin;
      /* avoid dividing valley starting at 0, starts at 45 degrees */
      /* necessary but not sufficient condition */ 
      sb += nsects/8;
      if (sb>=nsects)
	sb -= nsects;
      se += nsects/8;
      if (se>=nsects)
	se -= nsects;      
    }

    width = se-sb;
    if (width<0)
      width += nsects;

    split = nddHowManySplits(width, nsects, NDD_SPLIT_MAX_FRAC);
    
    if (split != 0) {
      nSplit++;
      nddSplitNValleys(diag, i, width/(split+1), split);
    }
  }

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: split %d valleys\n", nSplit);
#endif
  return nSplit;
}


static int nddFindGoalInValley(NDD_DIAGRAM * diag, int goal_sector) {
  int n, sb, se ;
  float width;
  double d1, d2, ang;

  for (n = 0; n<diag->n_valleys; n++) {
    sb = diag->valleys[n].begin;
    se = diag->valleys[n].end;
      
    if ( (se>=sb) &&
	 (goal_sector >= sb) && (goal_sector <= se) &&
	 ((width=nddEstimateValleyWidth(sb,se,diag, &d1, &d2, &ang)) 
	  > diag->lmax_robot) ){
      fprintf(stdout,"nddLib: width of selected valley: %5.2f (d1%5.2f d2%5.2f ang%5.2f)\n",
	      width, d1, d2, ang);
      return n;
    }
    if ( (sb>se) &&
	 ( (goal_sector >= sb) || (goal_sector <= se) ) &&
	 ((width = nddEstimateValleyWidth(sb,se,diag, &d1, &d2, &ang))
	  > diag->lmax_robot)){
      fprintf(stdout,"nddLib: width of selected valley: %5.2f (d1%5.2f d2%5.2f ang%5.2f)\n",
	      width, d1, d2, ang);
      return n;
    }
  }
  return -1;
}

static int nddFindGoalNearValley(NDD_DIAGRAM * diag, int goal_sector) {
  int n;
  int db,de, d1,d2;
  int best_valley1 = -1; 
  int best_valley2 = -1; 
  int best_to_goal1 = -1;
  int best_to_goal2 = -1;
  int current_to_goal;
  float width;
  /* for debug */
  float bestWidth1, bestWidth2;
  double d1_0, d2_0, ang_0;
  double d1_1, d2_1, ang_1;
  double d1_2, d2_2, ang_2;
  
  for (n = 0; n<diag->n_valleys; n++) {
    if ((width=nddEstimateValleyWidth(diag->valleys[n].begin,
				      diag->valleys[n].end,
				      diag, &d1_0, &d2_0, &ang_0))
	> (diag->lmax_robot*1.2)) {
      db = nddSectorDistance(diag->valleys[n].begin, goal_sector, diag->nsectors);
      de = nddSectorDistance(diag->valleys[n].end, goal_sector, diag->nsectors);
      
      if (db<de) current_to_goal = db;
      else       current_to_goal = de;

      if (best_to_goal1 == -1) { 
 	best_to_goal1 = current_to_goal; 
 	best_valley1 = n; 
	bestWidth1 = width;
	d1_1 = d1_0; d2_1 = d2_0; ang_1 = ang_0;
      } else if (best_to_goal2 == -1) { 
 	best_to_goal2 = current_to_goal; 
 	best_valley2 = n;  
	bestWidth2 = width;
	d1_2 = d1_0; d2_2 = d2_0; ang_2 = ang_0;
      } else if ( (best_to_goal1 < best_to_goal2) && 
 		  (current_to_goal < best_to_goal2) ) {
 	best_to_goal2 = current_to_goal; 
 	best_valley2 = n;  
	bestWidth2 = width;
	d1_2 = d1_0; d2_2 = d2_0; ang_2 = ang_0;
      } else if ( (best_to_goal2 < best_to_goal1) && 
 		  (current_to_goal < best_to_goal1) ) {
 	best_to_goal1 = current_to_goal; 
 	best_valley1 = n; 
	d1_1 = d1_0; d2_1 = d2_0; ang_1 = ang_0;
	bestWidth1=width;
      }

 
    } 
  } 

/*   if (best_valley1 == -1){ */
/*     printf("nddLib: width of selected valley: %5.2f (d1 %5.2f d2 %5.2f ang %5.2f)\n", bestWidth2, d1_2, d2_2, ang_2); */
/*     return best_valley2;  */
/*   } */
  if (best_valley2 == -1) { /* only one valley */
    printf("nddLib: width of selected valley: %5.2f (d1 %5.2f d2 %5.2f ang %5.2f)\n", bestWidth2, d1_1, d2_1, ang_1);
    return best_valley1; 
  }


  d1 = nddSectorDistance(best_to_goal1, goal_sector, diag->nsectors);
  d2 = nddSectorDistance(best_to_goal2, goal_sector, diag->nsectors);


#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: findGoalNearValley: d1=%d d2=%d d1-d2=%d\n",
	  d1, d2, abs(d1-d2));
#endif

  if (abs(d1-d2) < (diag->nsectors/6)) {
    db = nddSectorDistance(best_to_goal1, 0, diag->nsectors); 
    de = nddSectorDistance(best_to_goal2, 0, diag->nsectors); 
  } else {
    db = nddSectorDistance(best_to_goal1, goal_sector, diag->nsectors);  
    de = nddSectorDistance(best_to_goal2, goal_sector, diag->nsectors); 
  }

  if (db<=de) {
    printf("nddLib: width of selected valley: %f\n", bestWidth1);
    return best_valley1; 
  }

  printf("nddLib: width of selected valley: %f\n", bestWidth2);
  return best_valley2; 
}


static void nddSelectValley(NDD_DIAGRAM * diag, NDD_POINT * local_goal) {
  int goal_sector;
  int goal_valley;
  
  goal_sector = nddGetGoalSector(local_goal, diag->nsectors);

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: goal_sector: %d\n", goal_sector);
#endif

  goal_valley = nddFindGoalInValley(diag, goal_sector);

  if (goal_valley == -1) {
#ifdef VERBOSE_NDDLIB
    fprintf(stdout, "nddLib: first goal_valley = %d\n", goal_valley);
#endif
    goal_valley = nddFindGoalNearValley(diag, goal_sector);
  }

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: goal_valley: %d: %d->%d\n", 
	  goal_valley, 
	  diag->valleys[goal_valley].begin,
	  diag->valleys[goal_valley].end);
#endif  

  diag->selected_valley = goal_valley;
}


static int nddSectorLowSafetyOneSide(NDD_DIAGRAM * diag, float * enlargement, 
			      double secuDist) {
  int sb,se, si,sj;
  int st;
  int nsects = diag->nsectors;
  int alpha, gamma;
  int sDisc;
  double dobs, secu;

  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;

  nddGetSiAndSjFromSelectedValley(diag, &si, &sj);
  if ( (sj == -1) ||
       (diag->rnd[sj] < diag->rnd[si]) )  {
    sDisc = sb;
    sj = si;
  } else
    sDisc = se;

  
  secu = nddGiveEnhacedSecurityDistance(sj,diag->nsectors,
					secuDist,
					diag->lmax_robot);

  dobs = nddEstimateDistToObsFromPND(sj, diag)/*  - secu */;
    
  alpha = nddComputeAlpha(diag->lmax_robot, 
			  secu,
			  nddEstimateDistToObsFromPND(sDisc, diag),
			  diag->nsectors);
  gamma = nddComputeGamma(dobs, sj,sDisc,
			  enlargement[sj],
			  alpha, diag->nsectors);
		
  if (nddSectorDistance(sb, sj, nsects) > nddSectorDistance(se, sj, nsects)) {
    if (alpha+gamma > diag->nsectors/4) 
      st = se - diag->nsectors/4; 
    else 
      st = se - (alpha+gamma);
    if (st < 0) st += nsects;
/*     if ( (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		       (double)nsects/NDDLIB_LIMIT_ANGLE/2.0) ) &&  */
/* 	 (st >= (int) ((double)nsects/NDDLIB_LIMIT_ANGLE/2.0)) ) */
/*       st = (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		  (double)nsects/NDDLIB_LIMIT_ANGLE/2.0) + 1; */
  } else {
    if (alpha+gamma > diag->nsectors/4) 
      st = sb + diag->nsectors/4; 
    else 
    st = sb + (alpha+gamma);
    if (st >= nsects) st -= nsects; 
/*     if ( (st >= (int) ((double)nsects/NDDLIB_LIMIT_ANGLE/2.0)) &&  */
/* 	 (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		       (double)nsects/NDDLIB_LIMIT_ANGLE/2.0) ) )	  */
/*       st = (int) ((double)nsects/NDDLIB_LIMIT_ANGLE/2.0) - 1; */
  }


#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: LS1: st=%d si=%d sj=%d \n", st, si, sj);
#endif

  return st;
}

static NDD_SECTOR nddSectorLowSafetyGoalValley(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
				 float * enlargement) {
  int si,sj;
  int st, goalSector, beta;

  goalSector = nddGetGoalSector(loc_goal, diag->nsectors),

  nddGetSiAndSjFromSelectedValley(diag, &si, &sj);
  if (diag->rnd[sj] < diag->rnd[si]) 
    sj = si;
      
  beta = nddComputeBeta(nddEstimateDistToObsFromPND(sj, diag), sj,
			goalSector, enlargement[sj], diag->nsectors);

  if (nddSectorDistance(diag->valleys[diag->selected_valley].begin,
			sj, diag->nsectors) 
      > nddSectorDistance(diag->valleys[diag->selected_valley].end,
			  sj, diag->nsectors)) {
    st = goalSector - beta;
    if (st < 0) st += diag->nsectors;
/*     if ( (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		       (double) diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) &&  */
/* 	 (st >= (int) (diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) ) */
/*       st = (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		  (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) + 1; */
  } else {
    st = goalSector + beta;
    if (st >= diag->nsectors) st -= diag->nsectors;
/*     if ( (st >= (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) &&  */
/* 	 (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		 (double) diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) ) */
/*       st = (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) - 1; */
  }

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: LSG: st = %d\n", st);
#endif

  return st;
}


static int nddSectorLowSafetyTwoSide(NDD_DIAGRAM * diag, float * enlargement, 
			      double secuDist) {
  int sb, se;
  int si, sj,  st, /*  c, */ nsects;
  double dobsi, dobsj, secuI, secuJ;
  int gamma; 
  int alpha, gi, gj;
  

  nsects = diag->nsectors;

  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;
  
  nddGetSiAndSjFromSelectedValley(diag, &si, &sj);

  dobsi = nddEstimateDistToObsFromPND(si,diag);
  dobsj = nddEstimateDistToObsFromPND(sj,diag);
  secuI = nddGiveEnhacedSecurityDistance(si,diag->nsectors, secuDist,
					 diag->lmax_robot);
  secuJ = nddGiveEnhacedSecurityDistance(sj,diag->nsectors, secuDist,
					 diag->lmax_robot);


  alpha = nddComputeAlpha(diag->lmax_robot, 
			  enlargement[sb], 
			  nddEstimateDistToObsFromPND(sb, diag), 
			  diag->nsectors);
  gi = nddComputeGamma(dobsi, si, sb,  
		       enlargement[si],
		       alpha, diag->nsectors);
  
  alpha = nddComputeAlpha(diag->lmax_robot, 
			  enlargement[se], 
			  nddEstimateDistToObsFromPND(se,diag), diag->nsectors); 
  gj = nddComputeGamma(dobsj, sj, se, enlargement[sj],alpha, diag->nsectors);
		       		     		        
  gamma = abs ((int) ((double)gi - (double)gj)/2); 

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: LS2: gamma=%d\n", gamma); 
#endif

  if (dobsi < dobsj)  {     
    st = nddGetBissector(sb,se,diag->nsectors) + gamma;  
    if (st >= (diag->nsectors)) st -= diag->nsectors;  
/*     if ( (st >= (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) &&   */
/*   	 (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)*  */
/*  		       (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) )   */
/*       st = (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) - 1;   */
  } else {  
    st = nddGetBissector(sb,se,diag->nsectors) - gamma;  
    if (st < 0) st += diag->nsectors;  
    /*     if ( (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)*  */
/*  		       (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) &&   */
/*   	 (st >= (diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) )   */
/*       st = (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)*  */
/*  		  (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) + 1;   */
  }  
  
    
#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: LS2: st = %d\n", st);
#endif

  return st;
}

static int nddSectorHighSafetyGoalValley(NDD_DIAGRAM * diag,  NDD_POINT * loc_goal) {
  int st;
  st = nddGetGoalSector(loc_goal, diag->nsectors);
#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: HSGV: st = %d\n", st);
#endif
  return st;
}

static int nddSectorHighSafetyWideValley(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
				  float * enlargement) {
  int sb, se, gs, db, de, st, alpha, sDisc;
  double dDisc;
/*   int i; */
  
  gs = nddGetGoalSector(loc_goal, diag->nsectors);

  se = diag->valleys[diag->selected_valley].end;
  sb = diag->valleys[diag->selected_valley].begin;

  db = nddSectorDistance(sb,gs,diag->nsectors);
  de = nddSectorDistance(se,gs,diag->nsectors);

  if (db < de) {
    dDisc = nddEstimateDistToObsFromPND(db, diag);
    sDisc = db;
  } else {
    dDisc = nddEstimateDistToObsFromPND(de,diag);
    sDisc = de;
  }

  alpha = nddComputeAlpha(diag->lmax_robot, 
			  enlargement[sDisc], dDisc, diag->nsectors);

  if (de < db)  {
    st =  se - alpha;
    if (st < 0) st += diag->nsectors;
/*     if ( (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		       (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) && */
/* 	 (st >= (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) ) */
/*       st = (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		  (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) + 1; */
  } else {
    st =  sb + alpha;
    if (st >= (diag->nsectors)) st -= diag->nsectors;
/*     if ( (st >= (int) ((double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0)) && */
/* 	 (st <= (int) ((NDDLIB_LIMIT_ANGLE*2.0-1.0)* */
/* 		       (double)diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) ) )  */
/*       st = (diag->nsectors/NDDLIB_LIMIT_ANGLE/2.0) - 1; */
  }

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: HSWV: st = %d\n", st);
#endif
  
  return st;
}

static int nddSectorHighSafetyNarrowValley(NDD_DIAGRAM * diag) {
  int st, sb, se;
  
  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;

  st = nddGetBissector(sb,se,diag->nsectors);

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: HSNV: st = %d\n", st);
#endif

  return st;
}


static double nddLinearSpeedHighSafety(double theta, double vmax, int linFactor) {
  double v;
  if ( (theta < (-M_PI/NDDLIB_LIMIT_ANGLE)) || 
       (theta > (M_PI/NDDLIB_LIMIT_ANGLE)))
    return 0.0;
  
  v = vmax * (1.0 - fabs(nddGiveSpeedFactorFromAngle(theta, linFactor))); 


  if (v>vmax)
    return vmax;
  return v;
}


static double nddLinearSpeedLowSafety(double theta, double vmax,
			       NDD_DIAGRAM * diag,
			       float security_distance,
				      float * enlargement, int linFactor) {
  float dobs, dEnlargement;
  int smin;
  double v;
  int sb,se, si,sj;
  double enhacedSecurityDistance;
  
  if ( (theta <= (-M_PI/NDDLIB_LIMIT_ANGLE)) ||
       (theta >= (M_PI/NDDLIB_LIMIT_ANGLE)))
    return 0.0;

  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;

  nddGetSiAndSjFromSector(diag, 0, &si, &sj);

  if (diag->rnd[si]<=diag->rnd[sj])
    smin = sj;
  else smin = si;

  dobs = nddEstimateDistToObsFromPND(smin, diag);

  dEnlargement = enlargement[smin];

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: LSn: dobs=%f, dEnlargement=%f\n", 
	  dobs,dEnlargement);
#endif

  enhacedSecurityDistance = nddGiveEnhacedSecurityDistance(smin,diag->nsectors,
							   security_distance,
							   diag->lmax_robot);
  if (dobs < (enhacedSecurityDistance)) {
    fprintf(stderr, "nddLib: WARNING: dobs < secu_dist\n");
/*     dobs = enhacedSecurityDistance + 0.05; */
    return 0.0;
  }

  if (dEnlargement < enhacedSecurityDistance) {
    fprintf(stderr, "nddLib: ERROR: dEnlargement < secu_dist\n");
    return 0.0;
/*     dEnlargement = enhacedSecurityDistance + 0.05; */
  }

  v = (vmax * (dobs  - enhacedSecurityDistance) / 
       (dEnlargement - enhacedSecurityDistance)       
       * (1.0 - fabs(nddGiveSpeedFactorFromAngle(theta, linFactor)))); 
    
  if (v>vmax) return vmax; 
  if (v<0.03) return 0.03;

  return v;
}

static double nddRotationnalVelocity(double theta, double wmax) {
  if (theta >= (M_PI/2))
    return wmax;
  if (theta <= (-M_PI/2)) 
    return -wmax; 
  return (2.0 * wmax * nddGiveSpeedFactorFromAngle(theta,1));
}


static int nddWhichHighSafety(NDD_DIAGRAM * diag, NDD_POINT *goal, int smax) {

  if (nddIsGoalInValley(diag, goal))
    return NDDLIB_STRAT_HSGV;

  if (nddIsWideValley(&(diag->valleys[diag->selected_valley]),
		      diag->nsectors,smax))
    return NDDLIB_STRAT_HSWV;

  if (nddIsNarrowValley(&(diag->valleys[diag->selected_valley]),
			diag->nsectors,smax))
    return NDDLIB_STRAT_HSNV;
  return NDDLIB_STRAT_NO;
}

static int nddChooseStrategy(NDD_DIAGRAM * diag, NDD_POINT * goal,
		      int smax) {
  int sb,se, bb, be,i;
  int side_one = 0;
  int side_two = 0;
  int nsects = diag->nsectors;

  if (diag->selected_valley == -1)
    return NDDLIB_STRAT_NO;
  
  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;    

  /*low safety, or obstacle behind? */

  nddGetBbAndBe(sb,se, nsects, &bb, &be);

  /*do we have an obstacle on the left?*/
  i = se;
  while (i!=be && !side_one) {
    if (diag->rnd[i] > diag->dmax_sensor)
      side_one = 1;
    i++;
    if (i >= nsects) i = 0;
  }

  /*do we have an obstacle on the right?*/
  i = sb;
  while (i!=bb && !side_two) {
    if (diag->rnd[i] > diag->dmax_sensor)
      side_two = 1;
    i--; 
    if (i < 0) i = nsects-1;
  }

#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: chooseStrategy: be=%d se=%d bb=%d sb=%d\n", 
	   be, se, bb, sb);
  if (side_one) fprintf(stdout, "nddLib: chooseStrategy: sideOne\n");
  if (side_two) fprintf(stdout, "nddLib: chooseStrategy: sideTwo\n");
#endif

  if (side_one || side_two) {
/*     if (nddIsGoalInValley(diag,goal))   */
/*       return NDDLIB_STRAT_LSG;   */
    if (side_one && side_two)
      return NDDLIB_STRAT_LS2;
    return NDDLIB_STRAT_LS1;
  }

  return nddWhichHighSafety(diag, goal, smax);
}

/* ----------------------------------------------------------------------
 * nddComputeRefGeneral
 *
 * return 1 if everything was OK or 0 in case of pb
 */

static int nddComputeRefGeneral( int fromPoints,
			  NDD_DIAGRAM * diag, NDD_POINT * points, 
			  int n_pt, 
			  NDD_SEGMENT * segments,	
			  int n_segs, 
			  NDD_POINT * lonelyPoints,
			  int n_lPoints,
			  double dMaxLocalMap,
			  float * enlargement,
			  double security_distance,
			  NDD_POINT * loc_goal,
			  double smax, double vmax,
			  double wmax,
				 double *thetaRef,
				 NDD_SPEED_REF * sr, int linFactor) {
  NDD_STRATEGY strat;
  NDD_SECTOR sector;
  double theta;
  double tmp;
  int res=1; /* 1 or 0 */

  if (fromPoints)
    nddFillPndFromPoints(diag, points, n_pt, 
			 sqrt(loc_goal->x*loc_goal->x+loc_goal->y*loc_goal->y));
  else {
    NDD_SEGMENT * newSegs;
    int newNSegs;
    float dist_to_goal;
    float dmax;
    dist_to_goal = sqrt (loc_goal->x * loc_goal->x  
		       + loc_goal->y * loc_goal->y);  
    if (dist_to_goal+diag->lmax_robot/2.0 < dMaxLocalMap)
      dmax = dist_to_goal+diag->lmax_robot/2.0;
    else
      dmax = dMaxLocalMap;
    newSegs = nddFiltrateSegments(segments, n_segs, dmax, &newNSegs);
    nddFillPndFromSegments(diag, newSegs, newNSegs, lonelyPoints, n_lPoints);
    free (newSegs);
  }

  nddFillRnd(diag, enlargement);
  nddFindValleys(diag, loc_goal);
  nddSplitValleys(diag); 
  nddSelectValley(diag, loc_goal);

  strat = nddChooseStrategy(diag, loc_goal, smax);
  diag->currentStrat = strat;
  
#ifdef VERBOSE_NDDLIB
  switch (strat) {
  case NDDLIB_STRAT_LS1:
    fprintf(stdout,"nddLib: LS1 chosen\n");
    break;
  case NDDLIB_STRAT_LSG:
    fprintf(stdout, "nddLib: LSG chosen\n"); 
    break; 
  case NDDLIB_STRAT_LS2:
    fprintf(stdout,"nddLib: LS2 chosen\n");
    break;
  case NDDLIB_STRAT_HSGV:
    fprintf(stdout,"nddLib: HSGV chosen\n");
    break;
  case NDDLIB_STRAT_HSWV:
    fprintf(stdout,"nddLib: HSWV chosen\n");
    break;
  case NDDLIB_STRAT_HSNV:
    fprintf(stdout,"nddLib: HSNV chosen\n");
    break;
  default:
    fprintf(stdout,"nddLib: no strategy chosen\n");
  }
#endif
    
  switch (strat) {
  case NDDLIB_STRAT_LS1:  
    sector = nddSectorLowSafetyOneSide(diag,enlargement, security_distance);
    break;
  case NDDLIB_STRAT_LSG:
    sector = nddSectorLowSafetyGoalValley(diag,loc_goal,enlargement); 
    break; 
  case NDDLIB_STRAT_LS2:  
    sector = nddSectorLowSafetyTwoSide(diag,enlargement,security_distance);   
    break;
  case NDDLIB_STRAT_HSGV: 
    sector = nddSectorHighSafetyGoalValley(diag, loc_goal);
    break;
  case NDDLIB_STRAT_HSWV:
    sector = nddSectorHighSafetyWideValley( diag, loc_goal,  enlargement);
    break;
  case NDDLIB_STRAT_HSNV: 
    sector = nddSectorHighSafetyNarrowValley(diag);
    break;
  default: 
    sector = -1;
  }

  if (sector == -1) {
    fprintf(stderr,"nddLib: sending rotating speedref\n");    
    sr->v = 0;
    sr->w = wmax/2;
    *thetaRef = M_PI/2.;
    return res;
  }

  theta = nddGiveAngleFromSector(sector, diag->nsectors);
  *thetaRef = theta;
  
  switch (strat)  {
  case NDDLIB_STRAT_LS1:
  case NDDLIB_STRAT_LS2:
  case NDDLIB_STRAT_LSG:
    tmp = nddLinearSpeedLowSafety (theta, vmax, diag, security_distance,
				   enlargement, linFactor); 
    if (!isnan(tmp))
      sr->v = tmp;
    else {
      sr->v = 0.0;
      fprintf(stderr,"nddLib: computed a NaN v ref\n");
      res = 0;
    }
    break;
  case NDDLIB_STRAT_HSNV:
  case NDDLIB_STRAT_HSWV:
  case NDDLIB_STRAT_HSGV: 
    tmp =  nddLinearSpeedHighSafety(theta,vmax, linFactor);
    if (!isnan(tmp))
      sr->v = tmp;
    else {
      sr->v = 0.0;
      fprintf(stderr,"nddLib: computed a NaN v ref\n");
      res = 0;
    }
    break;
  default: sr->v = 0.0;
  }

  tmp =  nddRotationnalVelocity(theta,wmax);
  if (!isnan(tmp))
    sr->w = tmp;
  else {
    sr->w = 0.0;
    fprintf(stderr,"nddLib: computed a NaN w ref\n");
    res = 0;
  }      

#ifdef VERBOSE_NDDLIB
  fprintf(stdout,"nddLib: v=%f; w=%f\n",sr->v, sr->w);
#endif
  return res;
}
				    
/* ----------------------------------------------------------------------
 * nddComputeRefFromPoints
 *
 * return 1 if everything was OK or 0 in case of pb
 */

int nddComputeRefFromPoints(NDD_DIAGRAM * diag, NDD_POINT * points, 
			    int n_pt, 
			    double dMaxLocalMap,
			    float * enlargement,
			    double security_distance, 
			    NDD_POINT * loc_goal,
			    double smax, double vmax,
			    double wmax, 				 
			    double *thetaRef,
			    NDD_SPEED_REF * sr, int linFactor) {
  
  return nddComputeRefGeneral( 1,  diag, points, n_pt, 
			       NULL, 0, NULL, 0, dMaxLocalMap,
			       enlargement, security_distance, loc_goal,
			       smax, vmax, wmax, thetaRef, sr, linFactor);   
}


/* ----------------------------------------------------------------------
 * nddComputeRefFromSegments
 *
 * return 1 if everything was OK or 0 in case of pb
 */
int nddComputeRefFromSegments(NDD_DIAGRAM * diag, 
			      NDD_SEGMENT * segments,	
			      int n_segs, 
			      NDD_POINT * lonelyPoints,
			      int n_lPoints,
			      double dMaxLocalMap,
			      float * enlargement,
			      double security_distance, 
			      NDD_POINT * loc_goal,
			      double smax, 
			      double vmax,
			      double wmax,
			      double *thetaRef,
			      NDD_SPEED_REF *sr, int linFactor) {

  return  nddComputeRefGeneral( 0,
				diag, NULL, 0,
				segments, n_segs, 
				lonelyPoints, n_lPoints,
				dMaxLocalMap,				    
				enlargement, security_distance, loc_goal,
				smax, vmax, wmax, thetaRef, sr, linFactor);
}


