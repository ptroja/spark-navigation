#include <stdlib.h>
#include <math.h>
#include "nddLibUtils.h"
#include "nddLib.h"
#include <stdio.h>

void nddFloatListFree(nddFloatList * list) {
  nddFloatList * fl;
  while (list != NULL) {
    fl = list->next;
    free(list);
    list = fl;
  }
}

float nddEstimateDistToObsFromPND(int i, NDD_DIAGRAM * diag) {

  int j,k;
  int nsectors = diag->nsectors;
  double dist1, dist2, dist;
  double r0, r1, r2;
  double x1, x2, y1, y2, x, y;
  double a,b;
  double p;  

  if (diag->pnd[i] != NDDLIB_PND_OCCUPIED)
    return  diag->dmax_sensor + diag->lmax_robot - diag->pnd[i];
  
  j = i+1;
  if (j == nsectors) j = 0;
  while (diag->pnd[j] == NDDLIB_PND_OCCUPIED) {
    j++;
    if (j == nsectors) j = 0;
  }
  k = i-1;
  if (k < 0) k = nsectors-1;
  while (diag->pnd[k] == NDDLIB_PND_OCCUPIED) {
    k--;
    if (k < 0)  k = nsectors-1;
  }

  dist1 = diag->dmax_sensor + diag->lmax_robot - diag->pnd[j]; 
  dist2 = diag->dmax_sensor + diag->lmax_robot - diag->pnd[k]; 

  r0 = ((double)i + 0.5) * 2.0 * M_PI / (double) nsectors; 
  if (r0>M_PI) r0 -= 2*M_PI;
  r1 = ((double)j + 0.5) * 2.0 * M_PI / (double) nsectors; 
  r2 = ((double)k + 0.5) * 2.0 * M_PI / (double) nsectors; 

  x1 = dist1 * cos(r1); 
  x2 = dist2 * cos(r2); 
  y1 = dist1 * sin(r1); 
  y2 = dist2 * sin(r2); 
  p = tan(r0);

  if (fabs(x1-x2) > NDDLIB_EPSILON_ESTIMATE_FROM_PND) {
    a = (y1-y2)/(x1-x2);
    if (y1<y2) 
      b = y1 - a*x1;
    else
      b = y2 - a*x2;
    x = b / (p-a);
  } else if (x1<x2)
    x = x1;
  else 
    x = x2;
  y = p * x;
  dist = sqrt(x*x+y*y);
  return dist;  
}

int nddSectorDistance (int d1, int d2, int n) {
  int ret;
  ret = abs(d1 - d2);
  if (ret > (n/2)) return (n - ret);
  else             return ret;
}


double nddEstimateDistToObstFromValues(nddFloatList* values_list) {
  double min;

  if (values_list == NULL)
    return NDDLIB_PND_FREE;
  min = values_list->v;
  if (min == NDDLIB_PND_OCCUPIED)
    return NDDLIB_PND_OCCUPIED;

  values_list = values_list->next;

  while ( (values_list != NULL) &&
	  (values_list->v != NDDLIB_PND_OCCUPIED) ) {    
    if (values_list->v < min)
      min = values_list->v;
    values_list = values_list->next;
  }
  if (values_list == NULL)
    return min;
  return NDDLIB_PND_OCCUPIED;
}


/****************************************************/
/*   findValleys utils                              */
/****************************************************/

void incJ (int *j, int modulo) {
  (*j)++;
  if ( (*j) == modulo )
    (*j) = 0;
}

void incIincJ(int *i, int *j, int modulo) {
  (*i) = (*j);
  incJ(j,modulo);
}

int loweringDisc (int i, int j, NDD_DIAGRAM * diag) {
  if ( (diag->pnd[i] - diag->pnd[j]) > diag->lmax_robot) 
    return 1;
  return 0;	
}

int raisingDisc(int i, int j, NDD_DIAGRAM * diag) {
  if ((diag->pnd[i] - diag->pnd[j]) < (-diag->lmax_robot))
    return 1;
  return 0;
}

/****************************************************/
/*   splitValleys utils                             */
/****************************************************/

void nddSplitNValleys(NDD_DIAGRAM *diag, int valleyToSplit, int width, 
		     int nSlices) {
  int dum, j;
  int se = diag->valleys[valleyToSplit].end;
  int nsects = diag->nsectors;

  dum = diag->valleys[valleyToSplit].begin + width;
  if (dum >= nsects) dum -= nsects;
  diag->valleys[valleyToSplit].end = dum;
  for (j=1; j<nSlices; j++) {
    diag->valleys[diag->n_valleys].begin = dum;
    dum += width;
    if (dum >= nsects) dum -= nsects;
    diag->valleys[diag->n_valleys++].end = dum;
  }
  diag->valleys[diag->n_valleys].begin = dum;
  diag->valleys[diag->n_valleys++].end = se;
}

int nddHowManySplits(int width, int nsectors, int maxFrac) {
  int i;
  for (i=(maxFrac-1); i>0; i--) {
    if ( (double)width>=((double)i*(double)nsectors/(double)maxFrac) ) 
      return i;
  }
  return 0;
}

/****************************************************/
/*   misc utils                                     */
/****************************************************/

int nddGetBissector(int s1, int s2, int nsects) {
  int bissector;
  bissector = (s1+s2)/2;

  if (! (  ( (s1<=s2) && (bissector<=s2) && (bissector>=s1) ) ||
	   ( (s1>s2)  && ( (bissector>=s1) ||  (bissector<=s2) ) ) ) ) {
    bissector += nsects/2;
    if (bissector >= nsects)
      bissector -= nsects;
  }
  return bissector;
}

void nddGetBbAndBe(int sb, int se, int nsects, int *bb, int *be) {
  int bissector;

  bissector = nddGetBissector(sb,se, nsects);
  *bb = bissector - nsects/5 - 1;
  if (*bb<0)
    *bb += nsects;    
  *be = bissector + nsects/5 + 1;
  if (*be>=nsects)
    *be -= nsects;
}

void nddGetSiAndSjGeneric(NDD_DIAGRAM * diag, int s1, int s2, int*si, int*sj) {
  /*take care! this function may return  values ==0 !!*/
  /*be sure that there is an obstacle somewhere!!*/
  int i;
  int bb, be;
  int nsects = diag->nsectors;

  nddGetBbAndBe(s1,s2,nsects, &bb, &be);

  i = s1;
  *si = -1;
  while (i!=bb) {
    if ( (*si == -1) ||
	 (diag->rnd[i] > diag->rnd[*si]) ) 
      /* &&(diag->rnd[i] != PND_FREE) ok */
      *si = i;
    i--; 
    if (i < 0) i = nsects-1;
  }

  i = s2;
  *sj = -1;
  while (i!=be) {
    if ( (*sj == -1) ||
	 (diag->rnd[i] > diag->rnd[*sj]) )
      /* &&(diag->rnd[i] != PND_FREE) ok */
      *sj = i;
    i++;
    if (i >= nsects) i = 0;
  }

}



void nddGetSiAndSjFromSelectedValley (NDD_DIAGRAM * diag, int*si, int*sj) {
  nddGetSiAndSjGeneric(diag,
		       diag->valleys[diag->selected_valley].begin,
		       diag->valleys[diag->selected_valley].end,
		       si,sj);
}


void nddGetSiAndSjFromSector(NDD_DIAGRAM * diag, int sector, int*si, int*sj) {
  nddGetSiAndSjGeneric(diag, sector, sector, si,sj);
}


double nddGiveAngleFromSector(NDD_SECTOR sector, int nSectors) {
  double angle;  
  angle = 2.0 * M_PI / (double) nSectors * ((double)sector+0.5);
  if (angle > M_PI)
    angle -= 2*M_PI;
  return angle;
}


double nddDistToSeg(NDD_SEGMENT *seg) {
  double x1,y1,x2,y2;
  double dist;

  x1 = seg->r1 * cos(seg->t1); 
  x2 = seg->r2 * cos(seg->t2); 
  y1 = seg->r1 * sin(seg->t1);
  y2 = seg->r2 * sin(seg->t2);
  
  dist = fabs(x1*(y2-y1)-y1*(x2-x1))
	 / sqrt( (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1) ) ;
  return dist;
}

NDD_SEGMENT * nddFiltrateSegments(NDD_SEGMENT * segments, int n_segs, 
				 double dmax,  int * newNSegs) {
  int i = 0;
  NDD_SEGMENT * newSegs = NULL;
  *newNSegs = 0;

  for (i=0; i<n_segs; i++) {
    if (nddDistToSeg(&(segments[i])) <= dmax) {
      newSegs = (NDD_SEGMENT*) realloc((void*)newSegs,
				       (*newNSegs+1)*sizeof(NDD_SEGMENT));
      newSegs[*newNSegs].r1 = segments[i].r1;
      newSegs[*newNSegs].t1 = segments[i].t1;
      newSegs[*newNSegs].r2 = segments[i].r2;
      newSegs[*newNSegs].t2 = segments[i].t2;
      (*newNSegs)++;
    }
  }
  return newSegs;
}

int nddComputeAlpha(double lmax, double secuDist, double dDisc, int nsects) {
  int alpha;
  if (secuDist > dDisc) {
    fprintf(stderr, "nddLib: computeAlpha: secuDist > dDisc !!!\n");
    alpha = nsects/4;
    fprintf(stdout, "nddLib: alpha=%d lmax=%f secuDist=%f dDisc=%f\n", 
	    alpha,lmax, secuDist, dDisc);
    return nsects/4;
  }
  alpha = (int) ( (asin(secuDist/dDisc) / 2.0/M_PI * (double) nsects));
#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: alpha=%d lmax=%f secuDist=%f dDisc=%f\n", 
	  alpha,lmax, secuDist, dDisc);
#endif
  return alpha;
}

int nddComputeGamma(double dObs, int sectObs,
		       int  sectDisc, double dEnlargement,
		       int alpha, int nsects) {
  int gamma;
  if (dEnlargement < dObs) {
    fprintf(stderr, "nddLib: ERROR : Compute gamma dE < dObs !!!!\n");  
    fprintf(stdout, "nddLib: gamma=%d dE=%f dObs=%f sectObs=%d sectDisc=%d\n",
	    gamma, dEnlargement, dObs, sectObs, sectDisc);
    return 0;
  }
  if (sectObs > nsects/2)
    sectObs -= nsects;
  if (sectDisc > nsects/2)
    sectDisc -= nsects;
  gamma = (int) ( (dEnlargement - dObs)/dEnlargement 
		 * fabs ( ((double) nsects/2.0 + (double) sectObs) - 
			  fabs((double) sectDisc - (double) alpha) ) );
#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: gamma=%d dE=%f dObs=%f sectObs=%d sectDisc=%d\n",
	  gamma, dEnlargement, dObs, sectObs, sectDisc);
#endif
  return gamma;
}

int nddComputeBeta(double dObs, int discObs,
		   int sectGoal, double dEnlargement, int nsects) {
  int beta;
  if (dEnlargement < dObs) {
    fprintf(stderr, "nddLib: ERROR : Compute beta dE < dObs !!!!\n");
    fprintf(stdout, "nddLib: beta = %d\n", beta);
    return 0;
  }
  if (discObs > nsects/2)
    discObs -= nsects;
  if (sectGoal > nsects/2)
    sectGoal -= nsects;
  beta = (int) ( (dEnlargement-dObs)/dEnlargement
		 * fabs ( ((double) nsects/2.0+ (double) discObs) - 
			  (double) sectGoal ) );
#ifdef VERBOSE_NDDLIB
  fprintf(stdout, "nddLib: beta = %d\n", beta);
#endif
  return beta;
}

int nddIsGoalInValley(NDD_DIAGRAM * diag, NDD_POINT * goal) {
  int goal_sector;
  int sb,se;

  goal_sector = nddGetGoalSector(goal, diag->nsectors);
  
  sb = diag->valleys[diag->selected_valley].begin;
  se = diag->valleys[diag->selected_valley].end;    
  
  if ( ((sb == se) &&  (goal_sector == sb) ) ||
       ( (sb > se) &&  ((goal_sector>=sb) || (goal_sector<=se)) ) ||
       ( (sb < se) && (goal_sector <= se) && (goal_sector >= sb)  ) )
    return 1;
  return 0;
}


int nddGetGoalSector(NDD_POINT * goal, int nsectors) {
  double angle;
  int dum;
  
  angle = atan2 (goal->y, goal->x); 
  if (angle<0) angle += 2*M_PI ;
  
  dum = (int) (angle / M_PI / 2.0 * (double) nsectors) ;
  if (dum >= nsectors) 
    return (dum - nsectors);
  return dum;
}


double nddGiveEnhacedSecurityDistance(int sector, int nSectors, 
				      double secuDist, double lmax_robot) {
  double cosangle, l2,e2;
  
  if ( (sector<=3*nSectors/4) && (sector>=nSectors/4) )
/*     return lmax_robot/2.0; */
    return 0.0;

  cosangle = cos(nddGiveAngleFromSector(sector, nSectors));
  l2 = lmax_robot*lmax_robot/4.0;
  e2 = 1.0-l2/secuDist/secuDist;
  /* we define an ellipse for the zero speed zone */
  return sqrt(l2/(1.0-e2*cosangle*cosangle));
}

double nddGiveSpeedFactorFromAngle(double theta, int linFactor) {
  int i;
  double factor = 1.0;
  for (i=0; i<linFactor; i++) 
    factor *= NDDLIB_LIMIT_ANGLE * theta / M_PI;  
  return factor;
}
