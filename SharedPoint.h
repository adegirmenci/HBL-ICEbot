#ifndef SHAREDPOINT_H
#define SHAREDPOINT_H

#include <QSharedDataPointer>

class SharedPointData;

class SharedPoint
{
public:
    SharedPoint();
    SharedPoint(double x, double y, double z);
    SharedPoint(const SharedPoint & rhs);
    SharedPoint &operator=(const SharedPoint &);
    ~SharedPoint();

    void setx(double pk);
    void sety(double pk);
    void setz(double pk);

    double getx();
    double gety();
    double getz();

    SharedPoint update(double px, double py, double pz);
    double dist(SharedPoint other);

    SharedPoint operator+(SharedPoint b) const;
    SharedPoint operator-(SharedPoint b) const;
    SharedPoint operator*(double k) const;
    SharedPoint operator/(double k) const;
    void move(double a, double b, double c);

signals:

public slots:

private:
    QSharedDataPointer<SharedPointData> data;
};

#endif // SHAREDPOINT_H
