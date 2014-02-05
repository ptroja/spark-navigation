/*  Smooth ND driver for Player/Stage
 *
 *  SND Authors:  Joey Durham (algorithm),
 *                Luca Invernizzi (driver implementation)
 *                Piotr Trojanek (code cleanup)
 *
 *  Implemented on top of Player - One Hell of a Robot Server
 *  Copyright (C) 2003  (Brian Gerkey, Andrew Howard)
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef SND_ALGO_H_
#define SND_ALGO_H_

#include <vector>
#include <utility>

#include "spaces.h"

class Gap {
    public:
        Angle bearing;
        double distance;
        int iDir;

        Gap();

        Gap(Angle ang, double d, int iD);
};

class Valley {
    public:
        Gap risingGap;
        Gap otherGap;

        Valley( Gap rG, Gap oG );
};

class Robot_Proxy;

class SND_algorithm
{
    private:
        Robot_Proxy & robot;

        const double robotRadius;
        const double minGapWidth;
        const double obstacleAvoidDist;
        const double maxSpeed;
        const double maxTurnRate;
        const double goalPositionTol;
        const double goalAngleTol;

        const double fMaxRange;
        const double fScanRes;
        uint iNumLPs;

        typedef std::vector<std::pair<double,Angle> > ragerScan_t;
        ragerScan_t ragerScan;
        typedef std::vector<Gap> gapVec_t;
        gapVec_t gapVec;
        Valley* pBestValley;
        double obsAvoidDelta;

        Angle driveAngle;

        bool isFilterClear( const Angle & testBearing, double width, double forwardLength, bool bDoRearCheck ) const;
        bool isRisingGapSafe( const Gap & risingGap ) const;
        void buildGapVector( );
        void removeDuplicateGaps( );
        void findBestValley( const Position & distToGoal );
        void setObsAvoidDelta( double safetyDist );

    public :
        SND_algorithm( Robot_Proxy & r );
        void step( );
};

#endif // SND_ALGO_H_
