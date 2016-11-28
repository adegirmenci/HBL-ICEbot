#ifndef SHAREDPOINT_H
#define SHAREDPOINT_H

#include <QSharedDataPointer>

class SharedPointData;

class SharedPoint
{
public:
    SharedPoint();
    SharedPoint(double x, double y, double z);
    SharedPoint(const SharedPoint& rhs);
    SharedPoint &operator=(const SharedPoint& rhs);
    ~SharedPoint();

    // modifiers
    void setx(double pk) { data->m_x = pk; }
    void sety(double pk) { data->m_y = pk; }
    void setz(double pk) { data->m_z = pk; }

    // accessors
    double getx() const { return data->m_x; }
    double gety() const { return data->m_y; }
    double getz() const { return data->m_z; }

    SharedPoint update(double px, double py, double pz);
    double dist(SharedPoint& other);

    SharedPoint operator+(const SharedPoint& b) const;
    SharedPoint operator-(const SharedPoint& b) const;
    SharedPoint operator*(const double k) const;
    SharedPoint operator/(const double k) const;
    void move(double a, double b, double c);

signals:

public slots:

private:
    QSharedDataPointer<SharedPointData> data;
};

#endif // SHAREDPOINT_H
