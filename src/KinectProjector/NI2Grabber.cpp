//
//  NI2Grabber.cpp
//  Magic-Sand
//
//  Created by Brett Renfer on 1/21/22.
//

#include "NI2Grabber.h"
#include "ofConstants.h"

NI2Grabber::NI2Grabber()
:newFrame(true),
bufferInitiated(false),
kinectOpened(false)
{
}

NI2Grabber::~NI2Grabber(){
    //    stop();
    waitForThread(true);
    
    device->exit();
    delete device;
}

/// Start the thread.
void NI2Grabber::start(){
    startThread(true);
}

/// Signal the thread to stop.  After calling this method,
/// isThreadRunning() will return false and the while loop will stop
/// next time it has the chance to.
void NI2Grabber::stop(){
    stopThread();
}

bool NI2Grabber::setup(){
    // settings and defaults
    storedframes = 0;
    ROIAverageValue = 0;
    setToGlobalAvg = 0;
    setToLocalAvg = 0;
    doInPaint = 0;
    doFullFrameFiltering = false;
    
    device = new ofxNI2::Device;
    device->setup();
    
    width = 320;
    height = 240;
    
//    kinect.setUseTexture(false);

    kinectDepthImage.allocate(width, height, 1);
    filteredframe.allocate(width, height, 1);
    kinectColorImage.allocate(width, height);
    kinectColorImage.setUseTexture(false);
    return openKinect();
}

bool NI2Grabber::openKinect() {
    
    bool bSetup = false;
    if (depth.setup(*device))
    {
        depth.setSize(width, height);
        depth.setFps(30);
        depth.start();
        bSetup = true;
    }
    
    if (color.setup(*device)) // only for kinect device
    {
        color.setSize(width, height);
        color.setFps(60);
        color.start();
        bSetup = true;
    } else {
        bSetup = false;
    }
    
    device->setEnableRegistration();  // To have correspondance between RGB and depth images
    return bSetup;
}
void NI2Grabber::setupFramefilter(int sgradFieldresolution, float newMaxOffset, ofRectangle ROI, bool sspatialFilter, bool sfollowBigChange, int snumAveragingSlots) {
    gradFieldresolution = sgradFieldresolution;
    ofLogVerbose("NI2Grabber") << "setupFramefilter(): Gradient Field resolution: " << gradFieldresolution;
    gradFieldcols = width / gradFieldresolution;
    ofLogVerbose("NI2Grabber") << "setupFramefilter(): Width: " << width << " Gradient Field Cols: " << gradFieldcols;
    gradFieldrows = height / gradFieldresolution;
    ofLogVerbose("NI2Grabber") << "setupFramefilter(): Height: " << height << " Gradient Field Rows: " << gradFieldrows;
    
    spatialFilter = sspatialFilter;
    followBigChange = sfollowBigChange;
    numAveragingSlots = snumAveragingSlots;
    minNumSamples = (numAveragingSlots+1)/2;
    maxOffset = newMaxOffset;

    //Framefilter default parameters
    maxVariance = 4 ;
    hysteresis = 0.5f ;
    bigChange = 10.0f ;
//    instableValue = 0.0;
    maxgradfield = 1000;
    initialValue = 4000;
//    outsideROIValue = 3999;
    minInitFrame = 60;
    
    //Setup ROI
    setKinectROI(ROI);
    
    //setting buffers
    initiateBuffers();
}

void NI2Grabber::initiateBuffers(void){
    filteredframe.set(0);

    averagingBuffer=new float[numAveragingSlots*height*width];
    float* averagingBufferPtr=averagingBuffer;
    for(int i=0;i<numAveragingSlots;++i)
        for(unsigned int y=0;y<height;++y)
            for(unsigned int x=0;x<width;++x,++averagingBufferPtr)
                *averagingBufferPtr=initialValue;
    
    averagingSlotIndex=0;
    
    /* Initialize the statistics buffer: */
    statBuffer=new float[height*width*3];
    float* sbPtr=statBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x)
            for(int i=0;i<3;++i,++sbPtr)
                *sbPtr=0.0;
    
    /* Initialize the valid buffer: */
    validBuffer=new float[height*width];
    float* vbPtr=validBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x,++vbPtr)
            *vbPtr=initialValue;
    
    /* Initialize the gradient field buffer: */
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
    
    bufferInitiated = true;
    currentInitFrame = 0;
    firstImageReady = false;
}

void NI2Grabber::resetBuffers(void){
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] gradField;
    }
    initiateBuffers();
}

void NI2Grabber::threadedFunction() {
    while(isThreadRunning()) {
        this->actionsLock.lock(); // Update the grabber state if needed
        for(auto & action : this->actions) {
            action(*this);
        }
        this->actions.clear();
        this->actionsLock.unlock();
        device->update();
        
//        kinect.update();
        if(color.isFrameNew()){
            
            kinectDepthImage = depth.getPixelsRef();
            filter();
            filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
            updateGradientField();
            kinectColorImage.setFromPixels(color.getPixelsRef());
        }
        if (storedframes == 0)
        {
            filtered.send(std::move(filteredframe));
            gradient.send(std::move(gradField));
            colored.send(std::move(kinectColorImage.getPixels()));
            lock();
            storedframes += 1;
            unlock();
        }
        
    }
//    kinect.close();
    color.exit();
    depth.exit();
    device->exit();
    delete[] averagingBuffer;
    delete[] statBuffer;
    delete[] validBuffer;
    delete[] gradField;
}

void NI2Grabber::performInThread(std::function<void(NI2Grabber&)> action) {
    this->actionsLock.lock();
    this->actions.push_back(action);
    this->actionsLock.unlock();
}

void NI2Grabber::filter()
{
    if (bufferInitiated && numAveragingSlots < 2)
    {
        // Just copy raw kinect data
        const RawDepth* inputFramePtr = static_cast<const RawDepth*>(kinectDepthImage.getData());
        float* filteredFramePtr = filteredframe.getData();
        inputFramePtr += minY*width;  // We only scan kinect ROI
        filteredFramePtr += minY*width;

        for (unsigned int y = minY; y < maxY; ++y)
        {
            inputFramePtr += minX;
            filteredFramePtr += minX;

            for (unsigned int x = minX; x < maxX; ++x, ++inputFramePtr, ++filteredFramePtr)
            {
                float newVal = static_cast<float>(*inputFramePtr);
                *filteredFramePtr = newVal;
            }
            inputFramePtr += width - maxX;
            filteredFramePtr += width - maxX;
        }

        if (doInPaint)
        {
            applySimpleOutlierInpainting();
        }

        if (spatialFilter)
        {
            applySpaceFilter();
        }
    }
    else if (bufferInitiated)
    {
        const RawDepth* inputFramePtr = static_cast<const RawDepth*>(kinectDepthImage.getData());
        float* averagingBufferPtr = averagingBuffer+averagingSlotIndex*height*width;
        float* statBufferPtr = statBuffer;
        float* validBufferPtr = validBuffer;
        float* filteredFramePtr = filteredframe.getData();
        
        inputFramePtr += minY*width;  // We only scan kinect ROI
        averagingBufferPtr += minY*width;
        statBufferPtr += minY*width*3;
        validBufferPtr += minY*width;
        filteredFramePtr += minY*width;

        for(unsigned int y=minY ; y<maxY ; ++y)
        {
            inputFramePtr += minX;
            averagingBufferPtr += minX;
            statBufferPtr += minX*3;
            validBufferPtr += minX;
            filteredFramePtr += minX;
            for(unsigned int x=minX ; x<maxX ; ++x,++inputFramePtr,++averagingBufferPtr,statBufferPtr+=3,++validBufferPtr,++filteredFramePtr)
            {
                float newVal = static_cast<float>(*inputFramePtr);
                float oldVal = *averagingBufferPtr;
                
                if(newVal > maxOffset)//we are under the ceiling plane
                {
                    *averagingBufferPtr = newVal; // Store the value
                    if (followBigChange && statBufferPtr[0] > 0){ // Follow big changes
                        float oldFiltered = statBufferPtr[1]/statBufferPtr[0]; // Compare newVal with average
                        if(oldFiltered-newVal >= bigChange || newVal-oldFiltered >= bigChange)
                        {
                            float* aaveragingBufferPtr;
                            for (int i = 0; i < numAveragingSlots; i++){ // update all averaging slots
                                aaveragingBufferPtr = averagingBuffer + i*height*width + y*width +x;
                                *aaveragingBufferPtr = newVal;
                            }
                            statBufferPtr[0] = numAveragingSlots; //Update statistics
                            statBufferPtr[1] = newVal*numAveragingSlots;
                            statBufferPtr[2] = newVal*newVal*numAveragingSlots;
                        }
                    }
                    /* Update the pixel's statistics: */
                    ++statBufferPtr[0]; // Number of valid samples
                    statBufferPtr[1] += newVal; // Sum of valid samples
                    statBufferPtr[2] += newVal*newVal; // Sum of squares of valid samples
                    
                    /* Check if the previous value in the averaging buffer was not initiated */
                    if(oldVal != initialValue)
                    {
                        --statBufferPtr[0]; // Number of valid samples
                        statBufferPtr[1] -= oldVal; // Sum of valid samples
                        statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
                    }
                }
                // Check if the pixel is "stable": */
                if(statBufferPtr[0] >= minNumSamples &&
                   statBufferPtr[2]*statBufferPtr[0] <= maxVariance*statBufferPtr[0]*statBufferPtr[0] + statBufferPtr[1]*statBufferPtr[1])
                {
                    /* Check if the new running mean is outside the previous value's envelope: */
                    float newFiltered = statBufferPtr[1]/statBufferPtr[0];
                    if(abs(newFiltered-*validBufferPtr) >= hysteresis)
                    {
                        /* Set the output pixel value to the depth-corrected running mean: */
                        *filteredFramePtr = *validBufferPtr = newFiltered;
                    } else {
                        /* Leave the pixel at its previous value: */
                        *filteredFramePtr = *validBufferPtr;
                    }
                }
                *filteredFramePtr = *validBufferPtr;
            }
            inputFramePtr += width-maxX;
            averagingBufferPtr += width-maxX;
            statBufferPtr += (width-maxX)*3;
            validBufferPtr += width-maxX;
            filteredFramePtr += width-maxX;
        }

        /* Go to the next averaging slot: */
        if(++averagingSlotIndex==numAveragingSlots)
            averagingSlotIndex=0;
        
        if (!firstImageReady){
            currentInitFrame++;
            if(currentInitFrame > minInitFrame)
                firstImageReady = true;
        }
        
        if (doInPaint)
        {
            applySimpleOutlierInpainting();
        }

        /* Apply a spatial filter if requested: */
        if(spatialFilter)
        {
            applySpaceFilter();
        }
    }
}

void NI2Grabber::setFullFrameFiltering(bool ff, ofRectangle ROI)
{
    doFullFrameFiltering = ff;
    if (ff)
    {
        setKinectROI(ofRectangle(0, 0, width, height));
    }
    else
    {
        setKinectROI(ROI);
        float *data = filteredframe.getData();

        // Clear all pixels outside ROI
        for (unsigned int y = 0; y < height; y++)
        {
            for (unsigned int x = 0; x < width; x++)
            {
                if (y < minY || y >= maxY || x < minX || x >= maxX)
                {
                    int idx = y * width + x;
                    data[idx] = 0;
                }
            }
        }

    }
}

void NI2Grabber::applySpaceFilter()
{
    for(int filterPass=0;filterPass<2;++filterPass)
    {
        // Pointer to first pixel of ROI
        float *ptrOffset = filteredframe.getData() + minY * width + minX;

        // Low-pass filter the values in the ROI
        // First a horisontal pass
        for(unsigned int x = 0; x < width; x++)
        {
            // Pointer to current pixel
            float* colPtr = ptrOffset + x;
            float lastVal = *colPtr;

            // Top border pixels
            *colPtr = (colPtr[0]*2.0f + colPtr[width]) / 3.0f;
            colPtr += width;
            
            // Filter the interior pixels in the column
            for(unsigned int y = minY+1; y < maxY-1; ++y, colPtr += width)
            {
                float nextLastVal = *colPtr;
                *colPtr=(lastVal + colPtr[0]*2.0f + colPtr[width])*0.25f;
                lastVal = nextLastVal; // To avoid using already updated pixels
            }
            
            // Filter the last pixel in the column:
            *colPtr=(lastVal + colPtr[0] * 2.0f)/3.0f;
        }

        // then a vertical pass
        for(unsigned int y = 0; y < height; y++)
        {
            // Pointer to current pixel
            float* rowPtr = ptrOffset + y * width;
            
            // Filter the first pixel in the row:
            float lastVal=*rowPtr;
            *rowPtr=(rowPtr[0]*2.0f + rowPtr[1]) / 3.0f;
            rowPtr++;
       
            // Filter the interior pixels in the row:
            for(unsigned int x = minX+1; x < maxX-1; ++x,++rowPtr)
            {
                float nextLastVal=*rowPtr;
                *rowPtr=(lastVal+rowPtr[0]*2.0f+rowPtr[1])*0.25f;
                lastVal=nextLastVal;
            }
            
            // Filter the last pixel in the row:
            *rowPtr=(lastVal+rowPtr[0]*2.0f)/3.0f;
        }
    }
}

void NI2Grabber::updateGradientField()
{
    int ind = 0;
    float gx;
    float gy;
    int gvx, gvy;
    float lgth = 0;
    float* filteredFramePtr=filteredframe.getData();
    for(unsigned int y=0;y<gradFieldrows;++y) {
        for(unsigned int x=0;x<gradFieldcols;++x) {
            if (isInsideROI(x*gradFieldresolution, y*gradFieldresolution) && isInsideROI((x+1)*gradFieldresolution, (y+1)*gradFieldresolution) ){
                gx = 0;
                gvx = 0;
                gy = 0;
                gvy = 0;
                for (unsigned int i=0; i<gradFieldresolution; i++) {
                    ind = y*gradFieldresolution*width+i*width+x*gradFieldresolution;
                    if (filteredFramePtr[ind]!= 0 && filteredFramePtr[ind+gradFieldresolution-1]!=0){
                        gvx+=1;
                        gx+=filteredFramePtr[ind]-filteredFramePtr[ind+gradFieldresolution-1];
                    }
                    ind = y*gradFieldresolution*width+i+x*gradFieldresolution;
                    if (filteredFramePtr[ind]!= 0 && filteredFramePtr[ind+(gradFieldresolution-1)*width]!=0){
                        gvy+=1;
                        gy+=filteredFramePtr[ind]-filteredFramePtr[ind+(gradFieldresolution-1)*width];
                    }
                }
                if (gvx !=0 && gvy !=0)
                    gradField[y*gradFieldcols+x]=ofVec2f(gx/gradFieldresolution/gvx, gy/gradFieldresolution/gvy);
                if (gradField[y*gradFieldcols+x].length() > maxgradfield){
                    gradField[y*gradFieldcols+x].scale(maxgradfield);// /= gradField[y*gradFieldcols+x].length()*maxgradfield;
                    lgth+=1;
                }
            } else {
                gradField[y*gradFieldcols+x] = ofVec2f(0);
            }
        }
    }
}


float NI2Grabber::findInpaintValue(float *data, int x, int y)
{
    int sideLength = 5;

    // We do not search outside ROI
    int tminx = max(minX, x - sideLength);
    int tmaxx = min(maxX, x + sideLength);
    int tminy = max(minY, y - sideLength);
    int tmaxy = min(maxY, y + sideLength);

    int samples = 0;
    double sumval = 0;
    for (int y = tminy; y < tmaxy; y++)
    {
        for (int x = tminx; x < tmaxy; x++)
        {
            int idx = y * width + x;
            float val = data[idx];
            if (val != 0 && val != initialValue)
            {
                samples++;
                sumval += val;
            }
        }
    }
    // No valid samples found in neighboorhood
    if (samples == 0)
        return 0;

    return sumval / samples;
}

void NI2Grabber::applySimpleOutlierInpainting()
{
    float *data = filteredframe.getData();

    // Estimate overall average inside ROI
    int samples = 0;
    ROIAverageValue = 0;
    for (unsigned int y = minY; y < maxY; y++)
    {
        for (unsigned int x = minX; x < maxX; x++)
        {
            int idx = y * width + x;
            float val = data[idx];
            if (val != 0 && val != initialValue)
            {
                samples++;
                ROIAverageValue += val;
            }
        }
    }
    // No valid samples found in ROI - strange situation
    if (samples == 0)
        ROIAverageValue = initialValue;
    
    ROIAverageValue /= samples;

    setToLocalAvg = 0;
    setToGlobalAvg = 0;
    // Filter ROI
    for (unsigned int y = max(0, minY-2); y < min((int)height, maxY+2); y++)
    {
        for (unsigned int x = max(0, minX-2); x < min((int)width, maxX+2); x++)
        {
    //for (unsigned int y = minY; y < maxY; y++)
    //{
    //    for (unsigned int x = minX; x < maxX; x++)
    //    {
            int idx = y * width + x;
            float val = data[idx];

            if (val == 0 || val == initialValue)
            {
                float newval = findInpaintValue(data, x, y);
                if (newval == 0)
                {
                    newval = ROIAverageValue;
                    setToGlobalAvg++;
                }
                else
                {

                    setToLocalAvg++;
                }
                data[idx] = newval;
            }
        }
    }
}

bool NI2Grabber::isInsideROI(int x, int y){
    if (x<minX||x>maxX||y<minY||y>maxY)
        return false;
    return true;
}

void NI2Grabber::setKinectROI(ofRectangle ROI){
    if (doFullFrameFiltering)
    {
        minX = 0;
        maxX = width;
        minY = 0;
        maxY = height;
    }
    else
    { // we extend a bit beyond the border - to get data here as well due to shader issues
        minX = static_cast<int>(ROI.getMinX()) - 2;
        maxX = static_cast<int>(ROI.getMaxX()) + 2;
        minY = static_cast<int>(ROI.getMinY()) - 2;
        maxY = static_cast<int>(ROI.getMaxY()) + 2;
        
        minX = max(0, minX);
        maxX = min(maxX, (int)width);
        minY = max(0, minY);
        maxY = min(maxY, (int)height);
    }
    //ROIwidth = maxX-minX;
    //ROIheight = maxY-minY;
    resetBuffers();
}

void NI2Grabber::setAveragingSlotsNumber(int snumAveragingSlots){
    if (bufferInitiated){
            bufferInitiated = false;
            delete[] averagingBuffer;
            delete[] statBuffer;
            delete[] validBuffer;
            delete[] gradField;
        }
    numAveragingSlots = snumAveragingSlots;
    minNumSamples=(numAveragingSlots+1)/2;
    initiateBuffers();
}

void NI2Grabber::setGradFieldResolution(int sgradFieldresolution){
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] gradField;
    }
    gradFieldresolution = sgradFieldresolution;
    initiateBuffers();
}

void NI2Grabber::setFollowBigChange(bool newfollowBigChange){
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] gradField;
    }
    followBigChange = newfollowBigChange;
    initiateBuffers();
}

ofVec3f NI2Grabber::getStatBuffer(int x, int y){
    float* statBufferPtr = statBuffer+3*(x + y*width);
    return ofVec3f(statBufferPtr[0], statBufferPtr[1], statBufferPtr[2]);
}

float NI2Grabber::getAveragingBuffer(int x, int y, int slotNum){
    float* averagingBufferPtr = averagingBuffer + slotNum*height*width + (x + y*width);
    return *averagingBufferPtr;
}

float NI2Grabber::getValidBuffer(int x, int y){
    float* validBufferPtr = validBuffer + (x + y*width);
    return *validBufferPtr;
}

ofVec3f NI2Grabber::getNIWorldCoordinate(float x, float y, float wz)
{
    ofVec3f v;
    
//    const ofShortPixels& pix = depth.getPixelsRef();
//    const unsigned short *ptr = pix.getData();
//    unsigned short z = ptr[pix.getWidth() * y + x];
    
    openni::CoordinateConverter::convertDepthToWorld(depth.get(), x, y, wz, &v.x, &v.y, &v.z);

    return v;
}

ofMatrix4x4 NI2Grabber::getWorldMatrix() {
    auto mat = ofMatrix4x4();
    if (kinectOpened) {
        ofVec3f a = getNIWorldCoordinate(0, 0, 1);// Trick to access kinect internal parameters without having to modify ofxKinect
        ofVec3f b = getNIWorldCoordinate(1, 1, 1);
        ofLogVerbose("NI2Grabber") << "getWorldMatrix(): Computing kinect world matrix";
        mat = ofMatrix4x4(b.x - a.x, 0, 0, a.x,
            0, b.y - a.y, 0, a.y,
            0, 0, 0, 1,
            0, 0, 0, 1);
    }
    return mat;
}
