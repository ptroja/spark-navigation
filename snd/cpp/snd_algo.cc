/*  Smooth ND driver for Player/Stage
 *
 *  SND Authors:  Joey Durham (algorithm) ,
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
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <libplayerc++/playerc++.h>
#include <libplayercore/playercore.h>
#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>
//#include <boost/timer.hpp>

#include "snd_algo.h"
#include "snd_driver.h"
#include "spaces.h"

using namespace PlayerCc;
using namespace std;

Gap::Gap() :
   bearing(0.0),
   distance(0.0),
   iDir(0)
{}

Gap::Gap(Angle ang, double d, int iD) :
    bearing(ang),
    distance(d),
    iDir(iD)
{}

Valley::Valley( Gap rG, Gap oG ) :
    risingGap(rG),
    otherGap(oG)
{}

// Use gDebug to set verbosity of output, -1 for silent, 0 for hardly any output, 5 for normal debug
static int gDebug=1;

SND_algorithm::SND_algorithm( Robot_Proxy & r ) :
  robot(r),
  robotRadius(robot.robot_radius),
  minGapWidth(robot.min_gap_width),
  obstacleAvoidDist(robot.obstacle_avoid_dist),
  maxSpeed(robot.max_speed),
  maxTurnRate(robot.max_turn_rate),
  goalPositionTol(robot.goal_position_tol),
  goalAngleTol(robot.goal_angle_tol),
  fMaxRange(robot.GetMaxRange()),
  fScanRes(robot.GetScanRes()),
  iNumLPs(robot.GetCount()),
  pBestValley(NULL)
{
    if( gDebug > 0 )
    {
        std::cout << "Starting SND driver 3.0 ..." << std::endl;
        std::cout << "Robot radius: " << robotRadius << "; obstacle_avoid_dist " << obstacleAvoidDist << std::endl;
        std::cout << "Pos tol: " << goalPositionTol << "; angle tol " << goalAngleTol << std::endl;

        std::cout << "SND driver ready" << std::endl;

        std::cerr<< std::endl;
        std::cerr<< "Robot at " << robot.GetXPos() << ", " << robot.GetYPos() << std::endl;
        std::cerr<< std::endl;
    }
}

bool SND_algorithm::isFilterClear( const Angle & testBearing, double width, double forwardLength, bool bDoRearCheck ) const
{
    for( uint i = 0; i < ragerScan.size(); ++i )
    {
        double deltaAngle = ragerScan[i].second.alDiff(testBearing);

        if( std::abs(deltaAngle) > M_PI/2  )
        {
            // Semi-circle behind sensor
            if( bDoRearCheck && (ragerScan[i].first < width/2.0) )
            {
                if( gDebug > 7 ) std::cout<< "  Filter: obstacle at " << ragerScan[i].second.print() << " in rear semi-circle" << std::endl;
                return false;
            }
        }
        else
        {
            // Rectangle in front of robot
            double d1 = std::abs((width/2.0)/std::sin(deltaAngle));
            double d2 = std::abs(forwardLength/std::cos(deltaAngle));

            if( ragerScan[i].first < std::min(d1,d2) )
            {
                if( gDebug > 7 ) std::cout<< "  Filter: obstacle at " << ragerScan[i].second.print() << " in front rectangle" << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool SND_algorithm::isRisingGapSafe( const Gap & risingGap ) const
{
    // TODO: only checks if point creating gap is to close to obstacle on other side ...
    // does not guarantee safe passage through gap

    const Position posGapCorner = Position( risingGap.distance, risingGap.bearing );

    for( uint i = 0; i < ragerScan.size(); ++i )
    {
        double deltaAngle = risingGap.iDir * ragerScan[i].second.alDiff(risingGap.bearing);

        if( deltaAngle > 0.0 && deltaAngle < M_PI/2.0 )
        {
            if( ragerScan[i].first < fMaxRange - 0.01 )
            {
                if( posGapCorner.dist(Position(ragerScan[i].first, ragerScan[i].second)) < minGapWidth )
                {
                    if( gDebug > 4 ) cout << "    Gap at " << risingGap.bearing.print() << " ruled unsafe by obstacle at " << ragerScan[i].second.print() << endl;
                    return false;
                }
            }
        }
    }

    return true;
}

void SND_algorithm::buildGapVector( )
{
    ragerScan_t::value_type rayR, rayL;

    gapVec.clear();

    iNumLPs = robot.GetCount();

    if( iNumLPs == 0 )
    {
        return;
    }

    // Right edge of scan is right gap
    rayR = ragerScan[iNumLPs - 1];
    rayL = ragerScan[0];

    double dist = (rayR.first * rayR.first) + (rayL.first * rayL.first) - 2 * rayR.first * rayL.first * std::cos( rayR.second.ccwDiff(rayL.second) );

    if( dist >= minGapWidth )
    {
        Gap newGap( rayL.second, -robotRadius, -1 );
        gapVec.push_back(newGap);
    }

    for( uint i = 1; i < iNumLPs - 1; ++i )
    {
        rayR = rayL;
        rayL = ragerScan[i];

        dist = rayL.first - rayR.first;

        if( (dist >= minGapWidth) || (rayL.first == fMaxRange && rayR.first < fMaxRange) )
        {
            Gap newGap( rayR.second, rayR.first, -1 );
            gapVec.push_back(newGap);
        }
        else if( (dist <= -minGapWidth) || (rayR.first == fMaxRange && rayL.first < fMaxRange))
        {
            Gap newGap( rayL.second, rayL.first, 1 );
            gapVec.push_back(newGap);
        }
    }

    // Left edge of scan is a left gap
    rayR = ragerScan[iNumLPs - 1];
    rayL = ragerScan[0];

    dist = (rayR.first * rayR.first) + (rayL.first * rayL.first) - 2 * rayR.first * rayL.first * std::cos( rayR.second.ccwDiff(rayL.second) );

    if( dist >= minGapWidth )
    {
        Gap newGap( rayR.second, -robotRadius, 1 );
        gapVec.push_back(newGap);
    }
}

void SND_algorithm::removeDuplicateGaps( )
{
    std::vector<Gap>::iterator iterR, iterL;
    iterL = gapVec.begin();
    iterR = iterL;
    ++iterL;
    while( iterL != gapVec.end() && iterR != gapVec.end() && gapVec.size() > 1)
    {
        if( ((*iterR).bearing.ccwDiff((*iterL).bearing) < 2*fScanRes) && ((*iterL).iDir == 1) && ((*iterR).iDir == 1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*iterR).bearing.print() << endl;
            iterL = gapVec.erase(iterR);
        }

        iterR = iterL;
        ++iterL;
    }

    if( iterR != gapVec.end() && gapVec.size() > 1 )
    {
        iterL = gapVec.begin();

        if( ((*iterR).bearing.ccwDiff((*iterL).bearing) < 2*fScanRes) && ((*iterL).iDir == 1) && ((*iterR).iDir == 1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*iterR).bearing.print() << endl;
            gapVec.erase(iterR);
        }
    }

    std::vector<Gap>::reverse_iterator riterR, riterL;
    riterR = gapVec.rbegin();
    riterL = riterR;
    ++riterR;
    while( riterL != gapVec.rend() && riterR != gapVec.rend() && gapVec.size() > 1 )
    {
        if( ((*riterR).bearing.ccwDiff((*riterL).bearing) < 2*fScanRes) && ((*riterL).iDir == -1) && ((*riterR).iDir == -1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*(riterL + 1).base()).bearing.print() << endl;
            gapVec.erase((riterL+1).base());
        }

        riterL = riterR;
        ++riterR;
    }

    if( riterL != gapVec.rend() && gapVec.size() > 1 )
    {
        riterR = gapVec.rbegin();

        if( ((*riterR).bearing.ccwDiff((*riterL).bearing) < 2*fScanRes) && ((*riterL).iDir == -1) && ((*riterR).iDir == -1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*(riterL+1).base()).bearing.print() << endl;
            gapVec.erase((riterL+1).base());
        }
    }
}

void SND_algorithm::findBestValley( const Position & distToGoal )
{
    int iBestValleyRising = -1;
    int iBestValleyOther  = -1;

    for(gapVec_t::size_type iR = 0; iR < gapVec.size(); ++iR )
    {
        gapVec_t::size_type iL = (iR + 1)%(gapVec.size());

        int iRisingGap = -1;
        int iOtherGap = -1;

        if( gapVec[iR].iDir == -1 )
        {
            if( isRisingGapSafe(gapVec[iR]) )
            {
                iRisingGap = iR;
                iOtherGap = iL;
            }
            else
            {
                if( gDebug > 2 ) cout<< " Potential rising gap at " << gapVec[iR].bearing.print() << " is not safe" << endl;
            }
        }

        if( gapVec[iL].iDir == 1 )
        {
            if( isRisingGapSafe(gapVec[iL]))
            {
                if( iRisingGap >= 0 )
                {
                    // Both gaps are rising, pick one closer to goal

                    // Angular proximity
                    if( std::abs(gapVec[iL].bearing.alDiff(distToGoal.bearing())) < std::abs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) )
                    {
                        iRisingGap = iL;
                        iOtherGap = iR;
                    }

                    // Physical proximity
                    /*if( distToGoal.dist( Position(gapVec[iL].distance, gapVec[iL].bearing) ) <
                        distToGoal.dist( Position(gapVec[iR].distance, gapVec[iR].bearing) ) )
                    {
                        iRisingGap = iL;
                        iOtherGap = iR;
                    }*/
                }
                else
                {
                    iRisingGap = iL;
                    iOtherGap = iR;
                }
            }
            else
            {
                if( gDebug > 2 ) cout<< " Potential rising gap at " << gapVec[iL].bearing.print() << " is not safe" << endl;
            }
        }

        if( iRisingGap >= 0 )
        {
            if( iBestValleyRising >= 0 )
            {
                if( gDebug > 4 )
                {
                    cout<< "      Checking valley with rising " << iRisingGap << endl;
                    cout<< "        Goal at " << distToGoal.bearing().print() << endl;
                    cout<< "        diff from rising at " << gapVec[iRisingGap].bearing.print() << " is " << std::abs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) << endl;
                    cout<< "        diff from best at   " << gapVec[iBestValleyRising].bearing.print() << " is " << std::abs(gapVec[iBestValleyRising].bearing.alDiff(distToGoal.bearing())) << endl;
                }

                // Angular proximity
                if( std::abs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) < std::abs(gapVec[iBestValleyRising].bearing.alDiff(distToGoal.bearing())) )
                {
                    if( gDebug > 3 ) cout<< "    New best valley with rising " << iRisingGap << endl;
                    iBestValleyRising = iRisingGap;
                    iBestValleyOther = iOtherGap;
                }

                // Physical proximity
                /*if( distToGoal.dist( Position(gapVec[iRisingGap].distance, gapVec[iRisingGap].bearing) ) <
                    distToGoal.dist( Position(gapVec[iBestValleyRising].distance, gapVec[iBestValleyRising].bearing) ) )
                {
                    iBestValleyRising = iRisingGap;
                    iBestValleyOther = iOtherGap;
                }*/
            }
            else
            {
                if( gDebug > 3 ) cout<< "    New best valley with rising " << iRisingGap << endl;
                iBestValleyRising = iRisingGap;
                iBestValleyOther = iOtherGap;
            }
        }
    }

    if( iBestValleyRising >= 0 )
    {
        if( gDebug > 1 ) cout<< "  Best valley has rising gap at " << gapVec[iBestValleyRising].bearing.print() << endl;
        pBestValley = new Valley( gapVec[iBestValleyRising], gapVec[iBestValleyOther] );
    }
}

void SND_algorithm::setObsAvoidDelta( double safetyDist )
{
    obsAvoidDelta = 0.0;
    double deltaMag = 0.0;
    double deltaAngle = 0.0;
    double deltaAreaSum = 0.0;


    for( uint i = 0; i < ragerScan.size(); ++i )
    {
        if( ragerScan[i].first <= safetyDist + robotRadius )
        {
            deltaMag = limit((safetyDist + robotRadius - ragerScan[i].first)/safetyDist,0.0,1.0);
            deltaAngle = driveAngle.alDiff(ragerScan[i].second + M_PI);

            deltaAreaSum += deltaMag*deltaMag;

            obsAvoidDelta += deltaMag*deltaMag*deltaMag*deltaAngle;
        }
    }

    if( deltaAreaSum > 0 )
    {
        obsAvoidDelta /= deltaAreaSum;
    }
    else
    {
        obsAvoidDelta = 0.0;
    }
}


void SND_algorithm::step( )
{
    if (robot.PeekInputData() == false || robot.isNewGoalData() == false) {
        return;
    }

    double driveSpeed;
    double driveTurnRate = 0.0;

    // update _var values
    Pose robotPose = Pose( robot.GetXPos(), robot.GetYPos(), Angle(robot.GetYaw()) );
    if( gDebug > 2 ) cout << "pose " << robotPose.print() << endl;

    Pose goal = Pose( robot.goalX, robot.goalY, Angle(robot.goalA));
    if( gDebug > 2 ) cout << "goal " << goal.print() << endl;

    if( pBestValley != NULL )
    {
        delete pBestValley;
	pBestValley = NULL;
    }
    driveAngle = 0.0;
    obsAvoidDelta = 0.0;

    iNumLPs = robot.GetCount();

    if( iNumLPs <= 0 || iNumLPs > 100000 )
    {
        robot.SetSpeed( 0.0, 0.0 );
        return;
    }

    if( ragerScan.size() != iNumLPs )
    {
        ragerScan.resize( iNumLPs );
    }

    for( uint i = 0; i < iNumLPs; ++i )
    {
        ragerScan[i].first = robot.GetRange(i);
        ragerScan[i].second = Angle( fScanRes * (i - iNumLPs/2.0) );
    }

    Pose relGoal = (goal - robotPose);
    Position distToGoal = Position(relGoal.pos().norm(), relGoal.pos().bearing() - robotPose.ori());

    if( gDebug > 4 )
    {
        cout<< "  Rel goal pose " << relGoal.print() << endl;
        cout<< "  Dist to goal " << distToGoal.print() << endl;
    }

    // Goal position reached, no need to continue
    if( distToGoal.norm() <= goalPositionTol )
    {
        if( robotPose.ori().almostEqual( goal.ori(), goalAngleTol ) )
        {
            if( gDebug > 0 ) std::cout<< "Reached goal location " << goal.print() << std::endl;
            robot.SetSpeed( 0.0, 0.0 );

            robot.GoalReached();

            return;
        }
        else
        {
            if( robotPose.ori().ccwDiff(goal.ori()) < M_PI )
            {
                driveTurnRate = std::min( maxTurnRate/2, robotPose.ori().ccwDiff(goal.ori())/3 );
            }
            else
            {
                driveTurnRate = std::max( -maxTurnRate/2, robotPose.ori().cwDiff(goal.ori())/3 );
            }

            robot.SetSpeed( 0.0, driveTurnRate );

            return;
        }
    }

    // Check closest obstacle
    double distToClosestObstacle = fMaxRange;
    for( uint i = 0; i < iNumLPs; ++i )
    {
        if( ragerScan[i].first <= robotRadius )
        {
            if( gDebug > 0 ) cout<< " Obstacle inside of robot's boundary!  Stopping" << endl;
            robot.SetSpeed( 0.0, 0.0 );
            return;
        }

        if( ragerScan[i].first <= obstacleAvoidDist + robotRadius )
        {
            distToClosestObstacle = std::min( distToClosestObstacle, ragerScan[i].first );
        }
    }

    double safetyDist = limit( 3.0*(distToClosestObstacle - robotRadius), robotRadius/10.0, obstacleAvoidDist);

    // Create list of gap/discontinuity angles
    buildGapVector( );

    if( gDebug > 2 )
    {
        for( uint i = 0; i < gapVec.size(); ++i )
        {
            cout << "  Gap at " << gapVec[i].bearing.dCastPi() << ", dir " << gapVec[i].iDir;
            cout << ", " << gapVec[i].distance << endl;
        }
    }

    // Clean up gap list, combining neighboring gaps.  Keep right most right gap, left most left gap
    removeDuplicateGaps( );


    // Find best valley
    findBestValley( distToGoal );

    Angle safeRisingGapAngle;
    Angle midValleyAngle;

    // Drive scenarios
    if( isFilterClear( distToGoal.bearing(), minGapWidth, std::min(fMaxRange - robotRadius, (double)(distToGoal.norm())), false) )
    {
        if( gDebug > 0 ) cout<< "  Heading straight for goal, path is clear!" << endl;
        if( gDebug > 0 ) cout<< "   Dist to goal " << (double)distToGoal.norm() << " angle " << distToGoal.bearing().print() << endl;
        driveAngle = distToGoal.bearing().dCast();
    }
    else if( pBestValley == NULL )
    {
        if( gDebug > 0 ) cout<< "  No where to go, turning in place" << endl;
        driveAngle = M_PI/2.0;
        driveSpeed = 0.0;
        driveTurnRate = maxTurnRate/2.0;
        robot.SetSpeed( driveSpeed, driveTurnRate );
        return;
    }
    else
    {
        // arctangent seems to work better in tight situations, arcsin sometimes wanted to send robot far from valley
        // in tight situations
        Angle safetyDeltaAngle = std::atan2( obstacleAvoidDist + robotRadius, std::max(robotRadius, pBestValley->risingGap.distance) );
        /*
        Angle safetyDeltaAngle = M_PI/2;
        if( pBestValley->risingGap.distance > obstacleAvoidDist + robotRadius )
        {
            safetyDeltaAngle = asin( limit((obstacleAvoidDist + robotRadius)/(pBestValley->risingGap.distance), 0.0, 1.0) );
        }
        */

        if( gDebug > 1 )
        {
            cout<< "    Best valley has rising at " << pBestValley->risingGap.bearing.print() << endl;
            cout<< "      safety delta = " << safetyDeltaAngle.print() << endl;
        }

        safeRisingGapAngle = pBestValley->risingGap.bearing - (pBestValley->risingGap.iDir * safetyDeltaAngle.dCast());

        if( pBestValley->risingGap.iDir > 0 )
        {
            midValleyAngle = pBestValley->risingGap.bearing.cwMean(pBestValley->otherGap.bearing);
        }
        else
        {
            midValleyAngle = pBestValley->risingGap.bearing.ccwMean(pBestValley->otherGap.bearing);
        }

        if( std::abs(safeRisingGapAngle.alDiff(pBestValley->risingGap.bearing)) < std::abs(midValleyAngle.alDiff(pBestValley->risingGap.bearing)) )
        {
            driveAngle = safeRisingGapAngle;
        }
        else
        {
            driveAngle = midValleyAngle;
        }

        //if( gDebug > 1 )
        //{
            //cout<< "    Best valley has rising at " << pBestValley->risingGap.bearing.print() << endl;
            //cout<< "      safety delta = " << safetyDeltaAngle.print() << endl;
            //cout<< "      safe rising  = " << safeRisingGapAngle.print() << endl;
            //cout<< "      mid valley   = " << midValleyAngle.print() << endl;
        //}
    }

    // Consider nearby obstacles
    setObsAvoidDelta( safetyDist );

    if( gDebug > 2 ) cout<< " Starting drive angle " << driveAngle.print() << "   " << driveAngle.dCastPi() << endl;

    // Don't allow obstacles to change sign of sharp turns
    if( driveAngle.dCastPi() > M_PI/2.0 )
    {
        driveAngle += obsAvoidDelta;
        if( driveAngle.dCastPi() < 0.0 )
        {
            driveAngle = M_PI/2.0;
        }
    }
    else if( driveAngle.dCastPi() < -M_PI/2.0 )
    {
        driveAngle += obsAvoidDelta;
        if( driveAngle.dCast() > 0.0 )
        {
            driveAngle = -M_PI/2.0;
        }
    }
    else
    {
        driveAngle += obsAvoidDelta;
    }

    double theta = driveAngle.dCast() - (driveAngle.dCast() > M_PI ? 2*M_PI : 0);

    if( gDebug > 2 )
    {
        cout<< " Drive angle : " << driveAngle.print();
        cout<< " from mid " << midValleyAngle.print() << ", safe rising " << safeRisingGapAngle.print() << ", and " << (double)obsAvoidDelta << " obs delta" << endl;
    }

    if( gDebug > 2 ) cout<< " Theta " << theta << " , " << driveAngle.dCastPi() << endl;

    theta = limit( theta, -M_PI/2.0, M_PI/2.0 );

    driveTurnRate = maxTurnRate*(2.0*theta/M_PI);
    driveTurnRate *= limit(std::pow((double)distToGoal.norm(), 0.5), 0.2, 1.0);
    driveTurnRate *= limit(std::pow((double)(distToClosestObstacle - robotRadius)/obstacleAvoidDist, 0.5), 0.5, 1.0);

    theta = limit( theta, -M_PI/4.0, M_PI/4.0 );

    driveSpeed = maxSpeed;
    driveSpeed *= limit(std::pow((double)distToGoal.norm(), 0.5), 0.0, 1.0);
    driveSpeed *= limit(std::pow((double)(distToClosestObstacle - robotRadius)/obstacleAvoidDist, 0.5), 0.0, 1.0);
    driveSpeed *= limit((M_PI/6.0 - std::abs(theta))/(M_PI/6.0),0.0,1.0);

    robot.SetSpeed( driveSpeed, driveTurnRate );

    if( gDebug > 0 ) cout<< " Drive commands: " << driveSpeed << ", " << driveTurnRate << endl;
}
