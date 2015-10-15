#include "Point.h"

Point::Point()
{
    m_x = 0.0;
    m_y = 0.0;
    m_z = 0.0;
}

Point::Point(double x, double y, double z)
{
    m_x = x;
    m_y = y;
    m_z = z;
}

void Point::update(double px, double py, double pz)
{
    m_x = px;
    m_y = py;
    m_z = pz;
}

// Distance to another point.  Pythagorean thm.
double Point::dist(const Point other)
{
    double xd = m_x - other.m_x;
    double yd = m_y - other.m_y;
    double zd = m_z - other.m_z;
    return sqrt(xd*xd + yd*yd + zd*zd);
}

// Add or subtract two points.
Point Point::operator+(const Point& rhs)
{
    return Point(m_x + rhs.getx(),
                 m_y + rhs.gety(),
                 m_z + rhs.getz());
}
Point Point::operator-(const Point& rhs)
{
    return Point(m_x - rhs.getx(),
                 m_y - rhs.gety(),
                 m_z - rhs.getz());
}

Point Point::operator*(const double k)
{
    return Point(m_x*k, m_y*k, m_z*k);
}

Point Point::operator/(const double k)
{
    return Point(m_x/k, m_y/k, m_z/k);
}

Point& Point::operator=(const Point& rhs)
{
    m_x = rhs.getx();
    m_y = rhs.gety();
    m_z = rhs.getz();
    return *this;
}

// Move the existing point.
void Point::move(double a, double b, double c)
{
    m_x += a;
    m_y += b;
    m_z += c;
}
