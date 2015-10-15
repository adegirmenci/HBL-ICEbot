#include "SharedPoint.h"

class SharedPointData : public QSharedData
{
public:
    SharedPointData()
    {
        m_x = 0.0;
        m_y = 0.0;
        m_z = 0.0;
    }

    SharedPointData(double x, double y, double z)
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    SharedPointData(const SharedPointData &other)
            : QSharedData(other)
    {
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }

    ~SharedPointData() {}

    double m_x;
    double m_y;
    double m_z;
};

// Zero Constructor
SharedPoint::SharedPoint() : data(new SharedPointData) {}

// Constructor
SharedPoint::SharedPoint(double x, double y, double z)
{
    data( new SharedPointData(x, y, z) );
}

// Copy
SharedPoint::SharedPoint(const SharedPoint &rhs) : data(rhs.data) {}

// Copy
SharedPoint &SharedPoint::operator=(const SharedPoint &rhs)
{
    if (this != &rhs)
        data.operator=(rhs.data);
    return *this;
}

// Destructor
SharedPoint::~SharedPoint() {}

// modifiers
void SharedPoint::setx(double pk) { data->m_x = pk; }
void SharedPoint::sety(double pk) { data->m_y = pk; }
void SharedPoint::setz(double pk) { data->m_z = pk; }

// accessors
double SharedPoint::getx() { return data->m_x; }
double SharedPoint::gety() { return data->m_y; }
double SharedPoint::getz() { return data->m_z; }

SharedPoint SharedPoint::update(double px, double py, double pz)
{
    this->setx(px);
    this->sety(py);
    this->setz(pz);
    return this;
}

// Distance to another point.  Pythagorean thm.
double SharedPoint::dist(SharedPoint other) {
    double xd = this->getx() - other.getx();
    double yd = this->gety() - other.gety();
    double zd = this->getz() - other.getz();
    return sqrt(xd*xd + yd*yd + zd*zd);
}

SharedPoint SharedPoint::operator+(const SharedPoint b) const
{
    return SharedPoint(this->getx() + b.getx(),
                       this->gety() + b.gety(),
                       this->getz() + b.getz());
}
SharedPoint SharedPoint::operator-(const SharedPoint b) const
{
    return SharedPoint(this->getx() - b.getx(),
                       this->gety() - b.gety(),
                       this->getz() - b.getz());
}

SharedPoint SharedPoint::operator*(const double k) const
{
    return SharedPoint(this->getx()*k,
                       this->gety()*k,
                       this->getz()*k);
}

SharedPoint SharedPoint::operator/(const double k) const
{
    return SharedPoint(this->getx()/k,
                       this->gety()/k,
                       this->getz()/k);
}
