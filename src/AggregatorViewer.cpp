#include "AggregatorViewer.h"

namespace fog {

AggregatorViewer::AggregatorViewer(const std::string image) {
    frame = cv::imread(image);
    frame_width = frame.cols;
    frame_height = frame.rows;
    aspectRatio = (float)frame_width/(float)frame_height; 
}

void AggregatorViewer::init() {
    tk::gui::Viewer::init();
    glGenTextures(1, &frameTexture);
    glBindTexture(GL_TEXTURE_2D, frameTexture);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame_width, frame_height, 0, GL_BGR ,GL_UNSIGNED_BYTE, frame.data);

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
    glLineWidth(0.9);
}

void AggregatorViewer::draw() {
    tk::gui::Viewer::draw();
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
}

tk::common::Vector3<float> AggregatorViewer::convertPosition(int x, int y, float z) {
    float new_x = ((float)x/(float)frame_width - 0.5)*yScale;
    float new_y = -((float)y/(float)frame_height -0.5)*xScale;
    return tk::common::Vector3<float>{new_x, new_y, z};
}
}