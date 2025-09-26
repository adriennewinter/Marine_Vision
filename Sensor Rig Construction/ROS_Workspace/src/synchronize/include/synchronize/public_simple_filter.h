#ifndef PUBLIC_SIMPLE_FILTER_H
#define PUBLIC_SIMPLE_FILTER_H

#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages
#include <sensor_msgs/FluidPressure.h> // pressure sensor messages

template <typename M>

class PublicSimpleFilter : public message_filters::SimpleFilter<M>
{
public:
    void publicSignalMessage(const boost::shared_ptr<M const>& msg)
    {
        this->signalMessage(msg);
    }
};

#endif // PUBLIC_SIMPLE_FILTER_H