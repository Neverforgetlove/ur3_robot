#include "orbbec_camera.h"

Orbbec_Camera::Orbbec_Camera()
{
    Status rc = OpenNI::initialize();
    if(rc != STATUS_OK) {
        throw orbbec_exception("openni Initialize failed");
    }
    if(STATUS_OK!=m_device.open(ANY_DEVICE)) {
        throw orbbec_exception("open orbbec failed");
    }
    rc = m_depth.create(m_device, SENSOR_DEPTH);
    if(STATUS_OK != rc ) {
        throw orbbec_exception("depth stream create failed");
    }
    rc = m_color.create(m_device, SENSOR_COLOR);
    if(STATUS_OK != rc ) {
        throw orbbec_exception("color stream create failed");
    }
    m_depth.start();
    m_color.start();
    cout << "depth stream start" <<"color stream start"<< endl;
}

Orbbec_Camera::~Orbbec_Camera()
{
    m_depth.stop();
    m_depth.destroy();
    m_color.stop();
    m_color.destroy();
    m_device.close();
    OpenNI::shutdown();

}

Mat Orbbec_Camera::getColorImage()
{
    Mat color_src;
    VideoStream* color_streams = &m_color;
    VideoFrameRef color_frame;
    int readyColorStream = -1;
    for(int i =0;i<3;i++)
    {
        Status rc = OpenNI::waitForAnyStream(&color_streams, 1, &readyColorStream);
        if(readyColorStream == 0){
            m_color.readFrame(&color_frame);
            switch(color_frame.getVideoMode().getPixelFormat()){
            case PIXEL_FORMAT_RGB888:
                 const cv::Mat mImageRGB(color_frame.getHeight(), color_frame.getWidth(), 
                                CV_8UC3, (void*)color_frame.getData() );
                cv::cvtColor( mImageRGB, color_src, CV_RGB2BGR );
                break;
            }
            if(color_src.empty()){
                break;
            }
        }
    }
    return color_src.clone();
}


Mat Orbbec_Camera::getDepthImage()
{
    Mat src;
    VideoStream* streams = &m_depth;
    VideoFrameRef frame;
    int readyStream = -1;
    for(int i=0; i<3; i++) {
        Status rc = OpenNI::waitForAnyStream(&streams, 1, &readyStream);
        if(readyStream == 0) {
            m_depth.readFrame(&frame);
            switch(frame.getVideoMode().getPixelFormat()) {
            case PIXEL_FORMAT_DEPTH_1_MM:
                src = Mat (	frame.getHeight(),
                            frame.getWidth(),
                            CV_16UC1,
                            (void*)frame.getData()  );
                break;
            default:
                break;

            }
            if (src.empty()) {
                break;
            }
        }
    }
    return src.clone();

}


Device* Orbbec_Camera::get_device()
{
    return &m_device;
}

