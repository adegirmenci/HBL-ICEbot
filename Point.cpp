#include "Point.h"

Point::Point(double x, double y, double z) {
                m_x = x;
                m_y = y;
                m_z = z;
}

// helper functions - data access
void Point::setx(double pk) { m_x = pk; }
void Point::sety(double pk) { m_y = pk; }
void Point::setz(double pk) { m_z = pk; }

// helper functions - modifiers
const double Point::getx() const { return m_x; }
const double Point::gety() const { return m_y; }
const double Point::getz() const { return m_z; }

void Point::update(double px, double py, double pz)
{
    m_x = px;
    m_y = py;
    m_z = pz;
}

// Distance to another point.  Pythagorean thm.
double Point::dist(Point other) const
{
    double xd = m_x - other.m_x;
    double yd = m_y - other.m_y;
    double zd = m_z - other.m_z;
    return sqrt(xd*xd + yd*yd + zd*zd);
}

// Add or subtract two points.
Point Point::operator+(Point b) const
{
    return Point(m_x + b.m_x, m_y + b.m_y, m_z + b.m_z);
}
Point Point::operator-(Point b) const
{
    return Point(m_x - b.m_x, m_y - b.m_y, m_z - b.m_z);
}

Point Point::operator*(double k) const
{
    return Point(m_x*k, m_y*k, m_z*k);
}

Point Point::operator/(double k) const
{
    return Point(m_x/k, m_y/k, m_z/k);
}

// Move the existing point.
void Point::move(double a, double b, double c)
{
    m_x += a;
    m_y += b;
    m_z += c;
}
