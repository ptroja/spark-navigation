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
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include <iostream>
#include <sstream>

#include "spaces.h"

double Angle::alDiff(const Angle &other) const {
    return normPi(other.theta - theta);
}

double Angle::ccwDiff(const Angle &other) const {
    double otherTheta = theta <= other.theta ? other.theta : other.theta + 2.0*M_PI;
    return otherTheta - theta;
}

double Angle::cwDiff(const Angle &other) const {
    double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
    return other.theta - myTheta;
}


bool Angle::almostEqual(Angle A, double toll) const {
    double ad = alDiff(A);
    if ( fabs(ad) < toll ) return true;
    return false;
}

Angle Angle::nearestMean(const Angle &other, double w1) const {
    if (w1 < 0.0 || w1>1.0 ){
        std::cerr<< "(Angle) Weight exceeds range in weigthedCcwMean." << std::endl;
    }
    return Angle(theta + (1.0 - w1)*(alDiff(other)));
}

Angle Angle::ccwMean(const Angle &other) const {
    if(other.theta == theta ){
        return Angle(theta + M_PI);
    }
    double otherTheta = theta <= other.theta ? other.theta : other.theta + 2.0*M_PI;
    return Angle((otherTheta + theta)/2.0);
}

Angle Angle::cwMean(const Angle &other) const {

    double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
    return Angle((other.theta + myTheta)/2.0);
}


std::string Angle::print(std::streamsize precision) const {
    std::stringstream s;
    s.precision(precision);
    s.setf(std::ios::fixed,std::ios::floatfield);
    s << theta << " rad";
    return s.str();
}

// -------------------------------------------------------------------

/// \brief default constructor */
Position::Position() :
  _x(0.0),
  _y(0.0)
{}

/// \brief x y constructor */
Position::Position(double x_in,double y_in) :
  _x(x_in),
  _y(y_in)
{}

/// \brief theta constructor (gives sin() and cos() of theta)*/
Position::Position(Angle theta) { _x = std::cos(theta.dCast()); _y = std::sin(theta.dCast()); };

/// \brief d theta constructor */
Position::Position(double d,Angle theta) { _x=d*std::cos(theta.dCast()); _y=d*std::sin(theta.dCast()); };

/// \brief  Sets x coordinate.
void Position::setX (double x){
    _x = x;
}

/// \brief  Sets y coordinate.
void Position::setY (double y){
    _y= y;
}

/// \brief compound assignment operator += */
Position& Position::operator+=(const Position& a) {
    _x = _x + a._x;
    _y = _y + a._y;
    return *this;
}

/// \brief compound assignment operator -= */
Position& Position::operator-=(const Position& a) {
    _x = _x - a._x;
    _y = _y - a._y;
    return *this;
}

/// \brief compound assignment operator product by a scalar -* */
Position& Position::operator*=(const double scalar) {
    _x = _x*scalar;
    _y = _y*scalar;
    return *this;
}

/// \brief compound assignment operator division by a scalar -/ */
Position& Position::operator/=(const double scalar) {
    _x = _x/scalar;
    _y = _y/scalar;
    return *this;
}

/// \brief compound assignment operator sum by a scalar */
Position& Position::operator+=(const double scalar) {
    *this = Position(this->norm() + scalar, this->bearing());
    return *this;
}

/// \brief compound assignment operator sum by a Angle  */
Position& Position::operator+=(const Angle angle) {
    *this = Position(this->norm(), Angle(this->bearing() + angle));
    return *this;
}

/// \brief compound assignment operator sub by a scalar */
Position& Position::operator-=(const double scalar) {
    *this = Position(this->norm() - scalar, this->bearing());
    return *this;
}

/// \brief compound assignment operator sub by a Angle  */
Position& Position::operator-=(const Angle angle) {         
    *this = Position(this->norm(), Angle(this->bearing() - angle));
    return *this;
}

/// \brief  binary arithmetic operator + */
const Position Position::operator+(const double &other) const {
    return Position(*this) += other;
}

/// \brief  binary arithmetic operator - */
const Position Position::operator-(const double &other) const {
    return Position(*this) -= other;
}

/// \brief  binary arithmetic operator + */
const Position Position::operator+(const Angle &other) const {
    return Position(*this) += other;
}

/// \brief  binary arithmetic operator - */
const Position Position::operator-(const Angle &other) const {
    return Position(*this) -= other;
}

/// \brief  binary arithmetic operator + */
const Position Position::operator+(const Position &other) const {
    return Position(*this) += other;
}

/// \brief  binary arithmetic operator - */
const Position Position::operator-(const Position &other) const {
    return Position(*this) -= other;
}

/// \brief  \brief binary arithmetic operator product by a scalar
/// \note the scalar must be on the right side of the product
const Position Position::operator*(const double scalar) {
    return Position(*this) *= scalar;
}

/// \brief  \brief binary arithmetic operator division by a scalar
/// \note the scalar must be on the right side of the product
const Position Position::operator/(const double scalar) {
    return Position(*this) /= scalar;
}

/// \brief  compound operator == */
bool Position::operator==(const Position &other) const {
    if( (_x == other._x) && (_y == other._y) )
        return true;
    else return false;
}

/// \brief  compound operator != */
bool Position::operator!=(const Position &other) const {
    return !(*this == other);
}
/// \brief minimum with another position
Position Position::minimum(const Position& p) {
    return Position(std::min(_x,p._x),std::min(_y,p._y));
}

/// \brief maximum with another position
Position Position::maximum(const Position& p) {
    return Position(std::max(_x,p._x),std::max(_y,p._y));
}

/// \brief  coordinates */
double Position::x () const {
    return _x;
}


double Position::y () const {
    return _y;
}


/// \brief Norm of the vector.
/// \return the norm of the position vector
double Position::norm() const {
  return hypot(_y, _x);
}

/// \brief Gets the square of the norm of the vector.
/// \return The square norm of the position vector
double Position::squareNorm() const {
    return _y * _y + _x * _x;
}


/// \brief Distance between two vectors.
/// \param p Position in respect to which compute the distance
/// \return the norm of the difference between "this" and another passed Position
double Position::dist(Position p) const {
    Position d = (*this - p);
    return d.norm();
}


/// \brief Anomaly in polar coordinates
/// \return the bearing (phase or anomaly) of the position vector
Angle Position::bearing () const {
    return Angle( atan2(_y , _x) );
}


double Position::scalar(Position p) const {
    return _x*p.x() + _y*p.y();
}


/// \brief  print
std::string Position::print(std::streamsize precision) const {
    std::stringstream s;
    s.precision(3);
    s.setf(std::ios::fixed,std::ios::floatfield);
    s << "(" /*<< setprecision(precision)*/ << _x << " m, " /*<< setprecision(precision)*/ << _y << " m)" ;
    return s.str();
}


/** default constructor */
Pose::Pose() :
  _valid(false)
{}
/** double constructor */
Pose::Pose(double x, double y,Angle theta) :
  _pos(Position(x,y)),
  _ori(theta),
  _valid(true)
{}
/** Position,Angle constructor */
Pose::Pose(Position pos,Angle ori) :
  _pos(pos),
  _ori(ori),
  _valid(true)
{}
/**compound assignment operator += */
Pose& Pose::operator+=(const Pose& a) {
    _ori += a._ori;
    _pos += a._pos;
    return *this;
}
/**compound assignment operator -= */
Pose& Pose::operator-=(const Pose& a) {
    _ori -= a._ori;
    _pos -= a._pos;
    return *this;
}
/** binary arithmetic operator + */
const Pose Pose::operator+(const Pose &other) const {
    return Pose(*this) += other;
}
/** binary arithmetic operator - */
const Pose Pose::operator-(const Pose &other) const {
    return Pose(*this) -= other;
}
/** \brief binary arithmetic operator product by a Position (rototranslation)
*   \note the Position scalar must be on the right side of the product
*/
const Position Pose::operator*(Position p) {
    double x    =   (p.x()) * std::cos(_ori.dCast()) - (p.y())*std::sin(_ori.dCast()) + _pos.x();
    double y    =   (p.x()) * std::sin(_ori.dCast()) + (p.y())*std::cos(_ori.dCast()) + _pos.y();
    return Position(x,y);
}

bool Pose::operator==(const Pose &other) const {
    if( (_pos == other._pos) && (_ori == other._ori) )  
        return true;
    else return false;
}

bool Pose::operator!=(const Pose &other) const {
    return !(*this == other);
}

Position Pose::pos() const {
    return _pos;
}

Angle Pose::ori() const {
    return _ori;
}

void Pose::setX(double x) {
    _pos.setX(x);
}

void Pose::setY(double y) {
    _pos.setY(y);
}

void Pose::setOri(Angle ori){
    _ori = Angle(ori);
}
        
bool Pose::valid() const {
    return _valid;
}

std::string Pose::print(std::streamsize linPrecision, std::streamsize angPrecision) const {
    std::stringstream s;
    s << "(" <<  _pos.print(linPrecision) << ", " << _ori.print(angPrecision) << ")" ;
    return s.str(); 
}

void Pose::directComposition(Pose &t) {
    double cosTh = std::cos( t.ori().dCast() );
    double sinTh = std::sin( t.ori().dCast() );
    
    Position pos0(pos());
    // Angle ori0 = ori();

    (*this) = Pose(cosTh*pos0.x()-sinTh*pos0.y()+t.pos().x(), sinTh*pos0.x()+cosTh*pos0.y()+t.pos().y(), ori()+t.ori());
}
/** inverse composition */
void Pose::inverseComposition(Pose &t) {
    double cosAth = std::cos(t.ori().dCast());
    double sinAth = std::sin(t.ori().dCast());
    Pose temp(-t.pos().x()*cosAth-t.pos().y()*sinAth, +t.pos().x()*sinAth-t.pos().y()*cosAth, Angle(-t.ori().dCast()) );
    directComposition(temp);
}
    
/** change frame of non-applied pose*/
void Pose::uRotateFrame(Angle &t) {
//          ruota this.pos() di un angolo t lasciando invariato this.ori()
    double cosTh = std::cos( t.dCast() );
    double sinTh = std::sin( t.dCast() );
    
    Position pos0  ( this -> pos() );
    Angle ori0 ( this -> ori() );
    
    (*this) = Pose(cosTh*pos0.x() - sinTh*pos0.y() , sinTh*pos0.x() + cosTh*pos0.y() , ori0);
}

/** Inverse change of frame for non-applied pose*/
void Pose::uInverseRotateFrame(Angle &t) {
//          Position pos0;
//          Angle ori0;
    double cosTh = std::cos( t.dCast() );
    double m_sinTh = -std::sin( t.dCast() ); //così è come la rotazione diretta ma con t di segno cambiato
    Position pos0 ( this -> pos() );
    Angle ori0 ( this -> ori() );
    
    (*this) = Pose(cosTh*pos0.x() - m_sinTh*pos0.y(), m_sinTh*pos0.x() + cosTh*pos0.y(), ori0);
}
