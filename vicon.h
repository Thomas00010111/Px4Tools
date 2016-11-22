#ifndef VICON_H
#define VICON_H

#include "qstring.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class Vicon
{
private:

public:
    Vicon();
    void init();
    void setIpAndPort(QString ip_port);
    void enableVicon2VisionEstimate(bool enable, QString topic);
    void setParamsOrigin(QString paramPath);
    void request_viconData(QString subjectName, QString segmentName, int measurements);
};

#endif // VICON_H
