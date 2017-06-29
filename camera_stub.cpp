#include "../../camera.hpp"
#include "../../bv_processor.hpp"
#include "../../protocol.hpp"

#include "Simulator.hpp"

union Image image;

void camera_init()
{
}

void camera_captureImage()
{
    //note copyCameraImage is thread safe
    Simulator::getInstance()->getVizualization().copyCameraImage((uint8_t *) image.pixels);;
}

void camera_setMode(enum CAMERA_MODE mode)
{
}

short unsigned int camera_getImageHeight()
{
    return IMAGE_HEIGHT;
}

short unsigned int camera_getImageWidth()
{
    return IMAGE_WIDTH;
}

struct Pixel *camera_getImageData()
{
    return image.pixels;
}

void camera_sendImage()
{
//     printf("Got Image \n");

    protocol_sendDataRaw(IMAGE, (unsigned char *) camera_getImageData(), camera_getImageWidth() * camera_getImageHeight() * sizeof(struct Pixel));
}
