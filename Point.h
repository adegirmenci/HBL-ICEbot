#ifndef POINT_H
#define POINT_H


class Point
{
public:
    Point(double x = 0.0, double y = 0.0, double z=0.0);
    void setx(double pk);
    void sety(double pk);
    void setz(double pk);

    const double getx() const;
    const double gety() const;
    const double getz() const;

    void update(double px, double py, double pz);
    double dist(const Point other) const;

    Point operator+(const Point b) const;
    Point operator-(const Point b) const;
    Point operator*(const double k) const;
    Point operator/(const double k) const;
    void move(double a, double b, double c);

private:
    double m_x;
    double m_y;
    double m_z;
};

#endif // POINT_H
