#ifndef POINT_H
#define POINT_H

#include <math.h>

class Point
{
private:
    double m_x;
    double m_y;
    double m_z;

public:
    Point();
    Point(double x, double y, double z);
    ~Point(){}

    // helper functions - data access
    void setx(double pk) { m_x = pk; }
    void sety(double pk) { m_y = pk; }
    void setz(double pk) { m_z = pk; }

    // helper functions - modifiers
    double getx() const { return m_x; }
    double gety() const { return m_y; }
    double getz() const { return m_z; }

    void update(double px, double py, double pz);
    double dist(const Point other);

    Point operator+(const Point& rhs);
    Point operator-(const Point& rhs);
    Point operator*(const double k);
    Point operator/(const double k);
    Point& operator=(const Point& rhs);
    void move(double a, double b, double c);
};

#endif // POINT_H
