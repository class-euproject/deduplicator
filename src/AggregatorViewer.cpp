#include "AggregatorViewer.h"

namespace fog {

AggregatorViewer::AggregatorViewer(const std::string image, const bool to_show) {
    frame = cv::imread(image);
    show = to_show;
    frame_width = frame.cols;
    frame_height = frame.rows;
    aspectRatio = (float)frame_width/(float)frame_height; 
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    pthread_mutex_init(&mutex, &mutexattr);
    pthread_mutexattr_destroy(&mutexattr);
}

void AggregatorViewer::init() {
    if(show) {
        tk::gui::Viewer::init();
        glGenTextures(1, &frameTexture);
        //use fast 4-byte alignment (default anyway) if possible
        glPixelStorei(GL_UNPACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
        //set length of one complete row in data (doesn't need to equal image.cols)
        glPixelStorei(GL_UNPACK_ROW_LENGTH, frame.step/frame.elemSize());
        glBindTexture(GL_TEXTURE_2D, frameTexture);    
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame_width, frame_height, 0, GL_BGR ,GL_UNSIGNED_BYTE, frame.data);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void AggregatorViewer::draw() {
    if(show) {
        tk::gui::Viewer::draw();
        glLineWidth(6.0f);
        if(!V3D) //set 2D view
            tkViewport2D(width, height);
        //draw frame
        glPushMatrix(); {
            glTranslatef(0, 0, 0.001);
            glColor4f(1,1,1,1);
            xScale = xLim;
            yScale = xScale*aspectRatio;
            if(V3D)
                tkDrawTexture(frameTexture, frame_width, frame_height);
            else
                tkDrawTexture(frameTexture, xScale, yScale);
            
        } glPopMatrix();
        pthread_mutex_lock(&mutex);
        for(auto l:lines) {
            tkSetColor(l.color);
            tkDrawLine(l.points);
        }
        pthread_mutex_unlock(&mutex);
    }
}

void AggregatorViewer::setFrameData(const std::vector<tracker_line>& new_lines) { 
    pthread_mutex_lock(&mutex);
    lines = new_lines;
    pthread_mutex_unlock(&mutex);
}

tk::common::Vector3<float> AggregatorViewer::convertPosition2D(int x, int y, float z) {
    float new_x = ((float)x/(float)frame_width - 0.5)*yScale;
    float new_y = -((float)y/(float)frame_height -0.5)*xScale;
    return tk::common::Vector3<float>{new_x, new_y, z};
}

tk::common::Vector3<float> AggregatorViewer::convertPosition3D(int x, int y, float z) {
    float new_x = ((float)x/(float)frame_width - 0.5)*(float)frame_height;
    float new_y = -((float)y/(float)frame_height -0.5)*(float)frame_width;
    return tk::common::Vector3<float>{new_x, new_y, z};
}

void AggregatorViewer::tkViewport2D(int width, int height, int x, int y) {
    float ar = (float)width / (float)height;

    glViewport(x, y, width, height);
    glOrtho(0, width, 0, height, -1, 1);
    glLoadIdentity();
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();

    glOrtho(-ar, ar, -1, 1, 1, -1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

}