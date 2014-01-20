#ifndef SPACES_H
#define SPACES_H

/*  Smooth ND driver for Player/Stage
 *
 *  SND Authors:  Joey Durham (algorithm),
 *                Luca Invernizzi (driver implementation),
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
 *
 */

#include <cmath>
#include <cstring>

static const double M_2PI = 2.0*M_PI;  // For efficiency

/// Angle class borrowed from Multi-robot Integrated Platform
/// \author Antonio Franchi
class Angle {
    private:
        double theta;

        /// \brief Returns a value in [0, 2*M_PI).
        double norm2Pi(double a) const {
            if (a >= M_2PI || a < 0.0 ){
                a=std::fmod(a,M_2PI);            //in [-2*M_PI,2*M_PI]
                if (a<0.0)      a+=M_2PI;   //in [0,2*M_PI]
                if (a>=M_2PI) a-=M_2PI;
            }
            return a;
        }
        /// \brief Returns a value in (-M_PI, M_PI].
        double normPi(double a) const {
            if (a > M_PI || a <= -M_PI ){
                a=std::fmod(a,M_2PI);         //in [-2*M_PI,2*M_PI]
                if (a<=-M_PI) a+=M_2PI;
                if (a>M_PI)     a-=M_2PI;
            }
            return a;
        }
        /// \brief Returns a value in ]-M_2PI, 0].
        double normMinus2Pi(double a) const {
            if (a >=0.0 || a < -M_2PI ){
                a=std::fmod(a,M_2PI);            //in [-2*M_PI,2*M_PI]
                if (a<-M_2PI) a+=M_2PI;   //in [0,2*M_PI]
                if (a>=0.0)   a-=M_2PI;
            }
            return a;
        }

    public:
        /// \brief Default constructor.
        Angle() :
	  theta(0.0)
	{
        }
        /// \brief double constructor
        Angle(double t) :
	  theta(norm2Pi(t))
        {
        }
        /// \brief Copy constructor
        Angle(const Angle& rhs) :
	  theta(norm2Pi(rhs.dCast()))
        {
        }
        /// \brief Assigment operator =.
        Angle& operator=(const Angle& rhs){
            if (this != &rhs){      // Not necessary in this case but it is useful to not forget it
                theta = norm2Pi(rhs.dCast());
            }
            return *this;
        }
        /// \brief Compound assignment operator +=.
        Angle& operator+=(const Angle& a) {
            theta = norm2Pi(theta + a.theta);
            return *this;
        }
        /// \brief Compound assignment operator -=.
        Angle& operator-=(const Angle& other) {
            double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
            theta = norm2Pi(myTheta - other.theta);
            return *this;
        }
        /// \brief Compound assignment operator *=.
        Angle& operator*=(const Angle& a) {
            theta = norm2Pi(theta * a.theta);
            return *this;
        }
        /// \brief Compound assignment operator *= with scalar.
        Angle& operator*=(const double& d) {
            theta = norm2Pi(theta * d);
            return *this;
        }
        /// \brief Binary arithmetic operator +.
        const Angle operator+(const Angle &other) const {
            return Angle(*this) += other;
        }
        /// \brief Binary arithmetic operator -.
        const Angle operator-(const Angle &other) const {
            return Angle(*this) -= other;
        }
        /// \brief Binary arithmetic operator *.
        const Angle operator*(const Angle &other) const {
            return Angle(*this) *= other;
        }
        /// \brief Binary arithmetic operator * with scalar.
        const Angle operator*(const double& d) const {
            return (Angle(*this) *= d);
        }
        /// \brief Compound operator ==.
        bool operator==(const Angle &other) const {
            if( (other.dCast()) == theta)   return true;
            else return false;
        }
        /// \brief Compound operator !=.
        bool operator!=(const Angle &other) const {
            return !(*this == other);
        }
        /// \brief Explicit casting to double in (-M_PI, M_PI].
        double dCast() const {
            return theta;
        }
        /// \brief Explicit casting to double in (-M_PI, M_PI].
        double dCastPi() const {
            return normPi(theta);
        }
        /// \brief Explicit casting to double, in degrees [0, 360.0).
        double dCastDeg() const {
            return (theta*180.0)/M_PI;
        }

        /// \brief Algebric difference (in ]-PI,PI]).
        /// \details i.e, other - this in [0,2*PI[.
        /// \param[in] other The Angle respect to it is computed the algebric distance.
        /// \return The algebric distance respect to an angle, it is in [-PI,PI].
        double alDiff(const Angle &other) const;

        /// \brief Counter-clock-wise difference (in [0,2*PI[).
        /// \details i.e, other - this in in [0,2*PI[.
        /// \param[in] other The other angle.
        /// \return A double in [0,2*PI[.
        double ccwDiff(const Angle &other) const;

        /// \brief Clock-wise difference (in ]-2*PI,0]).
        /// \details i.e, other - this in in ]-2*PI,0].
        /// \param other The other angle.
        /// \return A double in ]-2*PI,0].
        double cwDiff(const Angle &other) const;

        /// \brief Equal condition with a tollerance.
        /// \param[in] A The Angle to be confronted with.
        /// \param[in] toll The acceppted tollerance (rad).
        /// \return \b true if difference betwen Angle is less than the tollerance, \b false otherwise.
        bool almostEqual(Angle A, double toll) const;

        /// \brief The same as ccwMean but weighted.
        /// \param[in].&other The Angle respect to it is computed the weigthed ccwMean.
        /// \param[in].w1 Weight of the this Angle. The weight of other is given as 1-w1.
        /// \note w1 must be in [0,1].
        /// \return An angle that is the weighted mean.
        Angle nearestMean(const Angle &other, double w1 = 0.5) const;

        /// \brief Counter-clock-wise mean, i.e, mean angle in ]this, other].
        /// \details If this == other, it is by definition this + M_PI.
        /// \param[in] other The other angle.
        /// \return Mean angle in ]this, other].
        Angle ccwMean(const Angle &other) const;

        /// \brief Clock-wise mean, i.e, mean angle in ]other, this].
        /// \details if this == other, it is by definition this.
        /// \param[in] other The other angle.
        /// \return Mean angle in ]other, this].
        Angle cwMean(const Angle &other) const;

        /// \brief Print function.
        /// \param[in] precision The number of doubles (default is 2).
        /// \return A printable string with the value printed on it.
        std::string print(std::streamsize precision=2) const;
};

/// Position class borrowed from Multi-robot Integrated Platform
/// \author Antonio Franchi
/// \brief Represents (x,y) in R^2.
class Position {

    private:

        double _x;
        double _y;

    public:
        /// \brief default constructor */
        Position();

        /// \brief x y constructor */
        Position(double x_in,double y_in);

        /// \brief theta constructor (gives sin() and cos() of theta)*/
        Position(Angle theta);

        /// \brief d theta constructor */
        Position(double d,Angle theta);

        /// \brief  Sets x coordinate.
        void setX (double x);

        /// \brief  Sets y coordinate.
        void setY (double y);

        /// \brief compound assignment operator += */
        Position& operator+=(const Position& a);

        /// \brief compound assignment operator -= */
        Position& operator-=(const Position& a);

        /// \brief compound assignment operator product by a scalar -* */
        Position& operator*=(const double scalar);

        /// \brief compound assignment operator division by a scalar -/ */
        Position& operator/=(const double scalar);

        /// \brief compound assignment operator sum by a scalar */
        Position& operator+=(const double scalar);

        /// \brief compound assignment operator sum by a Angle  */
        Position& operator+=(const Angle angle);

        /// \brief compound assignment operator sub by a scalar */
        Position& operator-=(const double scalar);

        /// \brief compound assignment operator sub by a Angle  */
        Position& operator-=(const Angle angle);

        /// \brief  binary arithmetic operator + */
        const Position operator+(const double &other) const;

        /// \brief  binary arithmetic operator - */
        const Position operator-(const double &other) const;

        /// \brief  binary arithmetic operator + */
        const Position operator+(const Angle &other) const;

        /// \brief  binary arithmetic operator - */
        const Position operator-(const Angle &other) const;

        /// \brief  binary arithmetic operator + */
        const Position operator+(const Position &other) const;

        /// \brief  binary arithmetic operator - */
        const Position operator-(const Position &other) const;

        /// \brief  \brief binary arithmetic operator product by a scalar
        /// \note the scalar must be on the right side of the product
        const Position operator*(const double scalar);

        /// \brief  \brief binary arithmetic operator division by a scalar
        /// \note the scalar must be on the right side of the product
        const Position operator/(const double scalar);

        /// \brief  compound operator == */
        bool operator==(const Position &other) const;

        /// \brief  compound operator != */
        bool operator!=(const Position &other) const;

        /// \brief minimum with another position
        Position minimum(const Position& p);

        /// \brief maximum with another position
        Position maximum(const Position& p);

        /// \brief  x coordinate
        double x () const;

        /// \brief  y coordinate
        double y () const;

        /// \brief Norm of the vector.
        /// \return the norm of the position vector
        double norm() const;

        /// \brief Gets the square of the norm of the vector.
        /// \return The square norm of the position vector
        double squareNorm() const;

        /// \brief Distance between two vectors.
        /// \param p Position in respect to which compute the distance
        /// \return the norm of the difference between "this" and another passed Position
        double dist(Position p) const;

        /// \brief Anomaly in polar coordinates
        /// \return the bearing (phase or anomaly) of the position vector
        Angle bearing () const;

        /// \brief Scalar product.
        /// \return The scalar product between *this and the input Position
        /// \param[in]&p The other position.
        double scalar(Position p) const;

        /// \brief  print
        std::string print(std::streamsize precision=4) const;
};

/// \author Antonio Franchi
/// \brief Represents (x,y,ori) in SE(2)
/// \todo Implementare le composizioni di pose.
/// \todo   Implementare le inversioni di pose.
class Pose {
    public:
        /// \brief  Default constructor.
        Pose();

        /// \brief  double constructor.
        Pose(double x, double y, Angle theta);

        /// \brief  Position,Angle constructor.
        Pose(Position pos, Angle ori);

        /// \brief compound assignment operator +=.
        Pose& operator+=(const Pose& a);

        /// \brief compound assignment operator -=.
        Pose& operator-=(const Pose& a);

        /// \brief  binary arithmetic operator +.
        const Pose operator+(const Pose &other) const;

        /// \brief  binary arithmetic operator -.
        const Pose operator-(const Pose &other) const;

        /// \brief binary arithmetic operator product by a Position (rototranslation).
        /// \note the Position scalar must be on the right side of the product
        const Position operator*(Position p);

        /// \brief  Compound operator ==.
        bool operator==(const Pose &other) const;

        /// \brief Compound operator !=.
        bool operator!=(const Pose &other) const;

        /// \brief Position.
        Position pos() const;

        /// \brief Orientation.
        Angle ori() const;

        /// \brief Set x.
        void setX(double);

        /// \brief Set x.
        void setY(double);

        /// \brief Set orientation.
        void setOri(Angle);

        /// \brief Validity.
        bool valid() const;

        /// \brief Print.
        std::string print(std::streamsize linPrecision=4, std::streamsize angPrecision=2) const;

        /// \brief  Direct composition.
        void directComposition(Pose &t);

        /// \brief Inverse composition.
        void inverseComposition(Pose &t);

        /// \brief Change frame of non-applied pose
        void uRotateFrame(Angle &t);

        /// \brief Inverse change of frame for non-applied pose
        void uInverseRotateFrame(Angle &t);

    private:

        Position _pos;
        Angle   _ori;
        bool   _valid; /** it is not valid with the default constructor, valid otherwise*/
};

#endif /* _SPACES_H */
