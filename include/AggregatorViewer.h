#ifndef AGGREGATORVIEWER_H
#define AGGREGATORVIEWER_H

#include <pthread.h>
#include "tkCommon/gui/Viewer.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "utils.h"

namespace fog {

struct tracker_line
{
    std::vector<tk::common::Vector3<float>> points;
    tk::gui::Color_t                        color;
};

class AggregatorViewer : public tk::gui::Viewer {
private:
    GLuint frameTexture;
    cv::Mat frame;
    bool newFrame = false;
    pthread_mutex_t mtxNewFrame;
    int frame_width, frame_height;
    float xScale, yScale;
    float aspectRatio;
    std::vector<tracker_line> lines;
    tk::common::Vector3<float> pose;
    tk::common::Vector3<float> size;
    pthread_mutex_t mutex;
public:
    bool show;
    
    AggregatorViewer(const std::string image, const bool to_show);
    ~AggregatorViewer() {}
    void init();
    void draw();
    void setFrameData(const std::vector<tracker_line>& new_lines);
    tk::common::Vector3<float> convertPosition2D(int x, int y, float z);
    tk::common::Vector3<float> convertPosition3D(int x, int y, float z);
    void tkViewport2D(int width, int height, int x, int y);
};
}

#endif /* AGGREGATORVIEWER_H */