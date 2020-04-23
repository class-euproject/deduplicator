#include "AggregatorViewer.h"

AggregatorViewer::AggregatorViewer(const std::string image) {
    frame = cv::imread(image);
    frame_width = frame.cols;
    frame_height = frame.rows;
    aspectRatio = (float)frame_width/(float)frame_height; 
}

void AggregatorViewer::init() {
    tk::gui::Viewer::init();
    glGenTextures(1, &frameTexture);
}

void AggregatorViewer::draw() {
    tk::gui::Viewer::draw();
    if(newFrame) {
        //when receiving a new frame, convert cv::Mat into GLuint texture
        glBindTexture(GL_TEXTURE_2D, frameTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,frame_width, frame_height, 0, GL_BGR,GL_UNSIGNED_BYTE, frame.data);
        
        pthread_mutex_lock(&mtxNewFrame);
        newFrame = false;
        pthread_mutex_unlock(&mtxNewFrame);
    }
    //set 2D view
    tkViewport2D(width, height);

    //draw frame
    glPushMatrix(); {
        glTranslatef(0, 0, 0.001);
        glColor4f(1,1,1,1);
        xScale = xLim;
        yScale = xScale*aspectRatio;
        tkDrawTexture(frameTexture, xScale, yScale);
    } glPopMatrix();

    for(auto l:lines) {
        tkSetColor(l.color);
        tkDrawLine(l.points);
    }
    
}

void AggregatorViewer::setFrameData(const std::vector<tracker_line>& new_lines) { 
    lines = new_lines;
    
    pthread_mutex_lock(&mtxNewFrame);
    newFrame = true;
    pthread_mutex_unlock(&mtxNewFrame);
}

tk::common::Vector3<float> AggregatorViewer::convertPosition(int x, int y, float z) {
    float new_x = ((float)x/(float)frame_width - 0.5)*yScale;
    float new_y = -((float)y/(float)frame_height -0.5)*xScale;
    return tk::common::Vector3<float>{new_x, new_y, z};
}