/* Copyright (C) 2015, DJI Innovations, Inc. All rights reserved.
 *
 * The software is licensed under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
 */
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include "cv.h"
#include "highgui.h"

//using namespace cv;

#ifndef __DJICAM_H
#define __DJICAM_H

#define DISPLAY_MODE            (1 << 0)
#define SAVEPICTURE_MODE        (1 << 1)
#define TRANSFER_MODE           (1 << 2)

extern IplImage * g_pImage;
#ifdef __cplusplus
extern "C" {
#endif
//void  Imagetransf(AVFrame *pFrame,int width,int height,int time);
int djiCam_init();
void djiCam_exit();
int djiCam_loop(IplImage* pImg);

#ifdef __cplusplus
}
IplImage* sendImageBack();
#endif

#endif

